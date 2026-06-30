/**
 ******************************************************************************
 * @file    robot.c
 * @brief   机械臂控制主逻辑实现文件
 *          包含任务创建、事件分发、PID 控制、关键运动与回零等核心功能
 ******************************************************************************
 */
#include "Emm_V5.h"
#include "bsp_uart.h"
#include "robot_kinematics.h"
#include <stdlib.h>
#include <math.h>
#include "robot_cmd.h"
#include "W800_mqtt.h"
#include "common_data.h"
#include "bsp_can.h"

extern CAN_Context_t g_can_context;

/* 最大插补路径点数：工作空间对角线约 500mm，分辨率 1mm，留 10% 余量 */
#define ROBOT_MAX_PATH_SIZE   (800)

/* 静态路径/逆解缓冲区，robot_control_task 单线程使用，消除运行时 malloc */
static struct position s_path_buf[ROBOT_MAX_PATH_SIZE];
static float           s_result_buf[ROBOT_MAX_PATH_SIZE * ROBOT_MAX_JOINT_NUM];

/* FreeRTOS 安全钩子 ---------------------------------------------------- */
void vApplicationMallocFailedHook(void)
{
    LOG("[FATAL] FreeRTOS malloc failed!\r\n");
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}
/* vApplicationStackOverflowHook 已在 src/robot_thread_entry.c 定义 */

struct robot g_robot;       /* robot 实例 */

/* 机械臂各关节的 DH 参数（长度单位 mm，角度单位 rad）*/
const float D_H[6][4] = {{0,        	0,          	0,          M_PI/2},
                         {0,        	M_PI/2,      	0,          M_PI/2},
                         {200,      	M_PI,        	0,         -M_PI/2},
                         {47.63f,    	-M_PI/2,     -184.5,    		 0},
                         {0,        	M_PI/2,      	0,          M_PI/2},
                         {0,        	M_PI/2,         75,         		 0}};

						 
/* 机械臂初始位置 T0_6 矩阵（位置单位 mm，方向余弦矩阵） */
const float T_0_6_reset[4][4] = {
    {0,    -1, 		0, 			0},
    {0, 	0,	   -1, 	  -47.63f},
    {1, 	0, 		0, 		15.5f},
    {0, 	0,		0, 			1},
};


/* 机械臂运动学仅控制 J1~J5，J6 由夹爪模块独立控制 */
#define ROBOT_ARM_JOINT_NUM (ROBOT_MAX_JOINT_NUM - 1)


/* 各关节权重：用于逆解结果选择时的加权误差计算（J6权重为0，不参与选解） */


/* 各关节默认初始化参数（角度、方向、减速比、限位实例、最小角、最大角、回零方向） */
static struct joint g_joints_init[ROBOT_MAX_JOINT_NUM] = {
	{90.0f,  MOTOR_DIR_CCW, 50.0f,  &JOINT_LIMIT_1,   0.0f, 360.0f, DIR_NEGATIVE, 0u, 0.0f, 0.0f},  /* 关节1 */
	{90.0f,  MOTOR_DIR_CW,  99.99f, &JOINT_LIMIT_2,  88.0f, 180.0f, DIR_NEGATIVE, 0u, 0.0f, 0.0f},  /* 关节 2 */
	{-90.0f, MOTOR_DIR_CW,  50.89f, &JOINT_LIMIT_3, -92.0f,  90.0f, DIR_NEGATIVE, 0u, 0.0f, 0.0f},  /* 关节 3 */
	{0.0f,   MOTOR_DIR_CW,  51.0f,  &JOINT_LIMIT_6, -90.0f,  90.0f, DIR_NEGATIVE, 0u, 0.0f, 0.0f},  /* 关节 4 */
	{90.0f,  MOTOR_DIR_CCW, 26.85f, &JOINT_LIMIT_5,   0.0f,  92.0f, DIR_POSITIVE, 0u, 0.0f, 0.0f},  /* 关节 5 */
	{0.0f,   MOTOR_DIR_CW,  51.0f,  &JOINT_LIMIT_4,   0.0f, 360.0f, DIR_NEGATIVE, 0u, 0.0f, 0.0f},  /* 关节 6 */
};


volatile struct robot_remote_control g_remote_control = {0};
static volatile bool g_soft_reset_done = false;
static volatile bool g_hard_reset_done = false;

static int robot_path_interpolation_scurve(struct position *target, int *size);

static int robot_update_current_angle(uint8_t joint_id);
static int robot_update_current_angle_retry(uint8_t joint_id, uint8_t retry_times);
static int robot_update_current_angle_from_data(uint8_t joint_id, uint8_t addr, const uint8_t *rx, uint8_t dlc);
static int robot_update_all_angles(uint8_t joint_num, uint32_t *missing_mask_out, uint32_t *elapsed_ms_out);
static int robot_angle_map_soft_reset(float angle, float min_angle, float max_angle, float *result);
static bool robot_joint_is_full_turn(uint8_t joint_id);
static int robot_joint_compare_angle(uint8_t joint_id, float raw_angle, float *compare_angle);
static int robot_joint_compare_error(uint8_t joint_id, float raw_angle, float ref_angle, float *err, float *compare_angle);
static int robot_joint_stop(uint8_t joint_id);
static void robot_joint_stop_all(uint8_t joint_num);
static int time_func_circle(uint32_t time_ms, struct position *pos);
static int robot_pid_run(struct position *path, int path_size, float *result);
static void robot_pid_one_period(float *target_angle, float *feedforward, float *total_error, int joint_num);
static int robot_pid_remote(void);
static int robot_mqtt_joints_sync(void);
static void robot_read_all_debug(void);
static void robot_joint_stop_from_isr(uint8_t joint_id);
static void robot_set_home_pose_valid(void);
static bool robot_verify_home_pose(uint8_t retry_times, float tol_deg, int *bad_joint, float *bad_err);
static bool robot_try_refresh_joints_feedback(uint8_t retry_times);
static float robot_angle_normalize(float angle);
static float robot_angle_diff(float cur_angle, float target_angle);
static void robot_joint_velocity_nowait(uint32_t joint_id, float velocity, uint8_t acceleration);


static robot_time_func g_robot_time_func = time_func_circle; /* 默认时间函数 */
static float g_time_func_circle_radius_mm = 15.0f;
#define ROBOT_CIRCLE_RADIUS_MAX_MM 20.0f

static void robot_set_home_pose_valid(void)
{
    g_robot.cur_pos.x = 0.0f;
    g_robot.cur_pos.y = 0.0f;
    g_robot.cur_pos.z = 0.0f;
    ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_POSE_VALID);
}

static bool robot_verify_home_pose(uint8_t retry_times, float tol_deg, int *bad_joint, float *bad_err)
{
    for (int i = 0; i < ROBOT_ARM_JOINT_NUM; i++) {
        if (robot_update_current_angle_retry((uint8_t)i, retry_times) != 0) {
            if (bad_joint) *bad_joint = i;
            if (bad_err) *bad_err = -1.0f;
            return false;
        }

        float cur = 0.0f;
        float err = 0.0f;
        if (robot_joint_compare_error((uint8_t)i, g_robot.joints[i].current_angle,
                g_joints_init[i].current_angle, &err, &cur) != 0) {
            if (bad_joint) *bad_joint = i;
            if (bad_err) *bad_err = -2.0f;
            return false;
        }

        g_robot.joints[i].current_angle = cur;
        if (err > tol_deg) {
            if (bad_joint) *bad_joint = i;
            if (bad_err) *bad_err = err;
            return false;
        }
    }

    return true;
}

static bool robot_try_refresh_joints_feedback(uint8_t retry_times)
{
    int fail = 0;

    for (int i = 0; i < ROBOT_ARM_JOINT_NUM; i++) {
        if (robot_update_current_angle_retry((uint8_t)i, retry_times) != 0) {
            fail++;
        }
    }

    if (fail > 0) {
        ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_POSE_DEGRADED);
        LOG("pose feedback degraded: %d/%d joints read failed, continue with estimated pose.\r\n", fail, ROBOT_ARM_JOINT_NUM);
        return false;
    }

    ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_POSE_DEGRADED);
    return true;
}

static void robot_joint_limit_post_handle(uint8_t joint_id)
{
	vTaskDelay(200); // 延时200ms等待限位开关释放
	taskENTER_CRITICAL();
	// 清除限位触发标志
	ROBOT_STATUS_CLEAR(g_robot.joints[joint_id].status, ROBOT_STATUS_LIMIT_HAPPENED);
	taskEXIT_CRITICAL();
}

uint32_t robot_joint_veloccity_to(uint32_t joint_id, float velocity, uint8_t acceleration)
{
	if (joint_id >= ROBOT_MAX_JOINT_NUM) {
        return 1;
    }

    struct joint *joint = &g_robot.joints[joint_id];
	
	uint8_t dir = (velocity > 0.0f) ? (uint8_t)joint->postive_direction : (uint8_t)(!joint->postive_direction);
	ROBOT_STATUS_CLEAR(joint->status, ROBOT_STATUS_LIMIT_ENABLE);
	uint8_t addr = (uint8_t)(joint_id + 1u); // 电机 CAN 地址以1 开始

	// 将速度值转换为电机所需的 RPM 单位并记录到关节状态
	joint->velocity = velocity;
	uint16_t _velocity = (uint16_t)fabsf(velocity * 600.0f * joint->reduction_ratio / 360.0f);
	
	// 发送速度控制命令（请求-应答需要串行化，否则会和夹爪/读角度互抢回包）
    if (!BSP_CAN_Lock(200)) {
        LOG("ERROR: CAN bus lock timeout on veloccity_to for joint:%d\n", joint_id);
        return 1;
    }

    Emm_V5_Vel_Control(addr, dir, _velocity, acceleration, false);

    uint8_t rx[8] = {0};
    uint8_t dlc = 0;
    if (!BSP_CAN_WaitReply((uint8_t)addr, 0xF6u, rx, &dlc, 50u, NULL))
    {
        BSP_CAN_Unlock();
        LOG("ERROR: CAN timeout on veloccity_to for joint:%d\n", joint_id);
        return 1;
    }

    BSP_CAN_Unlock();
	return 0;
}

/* PID 控制循环专用：发送速度命令但不等待回包（fire-and-forget）。
 * 参考 MechanicalArm Motor_VelControl，仅发送，靠下一周期读角度闭环校正。 */
static void robot_joint_velocity_nowait(uint32_t joint_id, float velocity, uint8_t acceleration)
{
    if (joint_id >= ROBOT_MAX_JOINT_NUM) {
        return;
    }
    struct joint *joint = &g_robot.joints[joint_id];
    uint8_t dir  = (velocity > 0.0f) ? (uint8_t)joint->postive_direction
                                     : (uint8_t)(!joint->postive_direction);
    uint8_t addr = (uint8_t)(joint_id + 1u);

    ROBOT_STATUS_CLEAR(joint->status, ROBOT_STATUS_LIMIT_ENABLE);
    joint->velocity = velocity;
    uint16_t rpm = (uint16_t)fabsf(velocity * 600.0f * joint->reduction_ratio / 360.0f);

    /* 与并行读角度共用总线，加锁但不等回包，发完立即释放 */
    if (!BSP_CAN_Lock(20u)) {
        return;
    }
    Emm_V5_Vel_Control(addr, dir, rpm, acceleration, false);
    BSP_CAN_Unlock();
}

/* 控制单关节旋转（支持相对与绝对两种模式） */
static uint32_t robot_joint_rotate_to(uint32_t joint_id, enum dir dir, float angle, float velocity, uint32_t acceleration, bool absolute)
{
	if (joint_id >= ROBOT_MAX_JOINT_NUM) {
		LOG("ERROR: joint_id is out of range");
        return 1;
    }

	if (velocity < 0) {
		LOG("ERROR: velocity is negative");
		return 1;
	}

    uint8_t joint_index = (uint8_t)joint_id;
   float rel_angle = 0;
	
   uint8_t _dir;
   struct joint *joint = &g_robot.joints[joint_index];

    if (absolute) {
        float target_angle = angle;

        if (robot_joint_is_full_turn(joint_index)) {
            target_angle = robot_angle_normalize(target_angle);
        } else {
            if (robot_joint_compare_angle(joint_index, target_angle, &target_angle) != 0) {
                LOG("ERROR: abs target angle out of range, joint:%u target:%.2f\r\n",
                    (unsigned)joint_id, angle);
                return 1;
            }
        }

        rel_angle = target_angle - joint->current_angle;
        if (robot_joint_is_full_turn(joint_index)) {
            if (rel_angle > 180.0f) {
                rel_angle -= 360.0f;
            } else if (rel_angle < -180.0f) {
                rel_angle += 360.0f;
            }
        }

		if (fabsf(rel_angle) < ROBOT_JOINT_ANGLE_ERROR_RANGE) {
			return 0;
		}

		_dir = (rel_angle >= 0.0f) ? (uint8_t)joint->postive_direction : (uint8_t)(!joint->postive_direction);
		rel_angle = fabsf(rel_angle);

		// LOG_FROM_ISR("id:%d current:%f, target:%f, rel:%f dir:%d\n", joint_id, joint->current_angle, angle, rel_angle, dir);
		joint->current_angle = target_angle;
	} else {
		/* 相对旋转分支：按给定角度增量改变当前角度 */
        rel_angle = angle;
        joint->current_angle += (dir == DIR_POSITIVE) ? angle : -angle;
		_dir = (dir == DIR_POSITIVE) ? (uint8_t)joint->postive_direction : (uint8_t)(!joint->postive_direction);
		if (rel_angle < 0) {
			_dir = !_dir;
		}
    }

	/* 计算需要旋转的步数并发送位置控制命令 */
	// 16 microstep * 200 步进分辨率示例，3200 步为一圈的细分值
    uint32_t steps = (uint32_t)fabsf(roundf(rel_angle * joint->reduction_ratio * 3200.0f / 360.0f));
	uint16_t _velocity = (uint16_t)fabsf(velocity * 600.0f * joint->reduction_ratio / 360.0f);
    uint8_t addr = (uint8_t)(joint_id + 1u); // 电机CAN地址从1开始
    ROBOT_STATUS_CLEAR(joint->status, ROBOT_STATUS_READY);
    Emm_V5_Pos_Control(addr, _dir, _velocity, (uint8_t)acceleration, steps, false, false);
    return 0;
}

static void robot_joint_limit_happend(uint8_t joint_id)
{
	if (g_robot.event_queue == NULL) {
        return;
    }

    // 检查限位开关是否启用
    if (!ROBOT_STATUS_IS(g_robot.joints[joint_id].status, ROBOT_STATUS_LIMIT_ENABLE)) {
        return;
    }

	// 检查是否已经触发过限位
    if (ROBOT_STATUS_IS(g_robot.joints[joint_id].status, ROBOT_STATUS_LIMIT_HAPPENED)) {
        return;
	}

	// 停止关节运动并标记限位触发
	robot_joint_stop_from_isr(joint_id);
	ROBOT_STATUS_SET(g_robot.joints[joint_id].status, ROBOT_STATUS_LIMIT_HAPPENED);

	struct robot_event event = {0};
	event.type = ROBOT_LIMIT_SWITCH_EVENT;
	event.joint_id = joint_id;
	BaseType_t xHigherPriorityTaskWoken;
    xQueueSendToBackFromISR(g_robot.event_queue, &event, &xHigherPriorityTaskWoken);
}

static const bsp_io_port_pin_t g_joint_limit_pins[ROBOT_MAX_JOINT_NUM] = {
	BSP_IO_PORT_08_PIN_04, /* joint 1 */
	BSP_IO_PORT_08_PIN_03, /* joint 2 */
	BSP_IO_PORT_08_PIN_02, /* joint 3 */
	BSP_IO_PORT_03_PIN_01, /* joint 4 */
	BSP_IO_PORT_03_PIN_02, /* joint 5 */
	BSP_IO_PORT_04_PIN_11, /* joint 6 */
};

static void robot_joint_limit_set_input(uint8_t joint_id)
{
	/* Disable external IRQ callback for this joint so it behaves as plain input */
	if (g_robot.joints[joint_id].p_limit_irq && g_robot.joints[joint_id].p_limit_irq->p_api)
	{
		g_robot.joints[joint_id].p_limit_irq->p_api->disable(g_robot.joints[joint_id].p_limit_irq->p_ctrl);
	}
}

static void robot_joint_limit_set_irq(uint8_t joint_id)
{
	/* Enable external IRQ callback for this joint */
	if (g_robot.joints[joint_id].p_limit_irq && g_robot.joints[joint_id].p_limit_irq->p_api)
	{
		g_robot.joints[joint_id].p_limit_irq->p_api->enable(g_robot.joints[joint_id].p_limit_irq->p_ctrl);
	}
}

static int robot_get_limit_status(uint8_t joint_id)
{
	/* Read pin level using IOPORT. Returns 1 if HIGH, 0 if LOW. */
	if (joint_id >= ROBOT_MAX_JOINT_NUM) return 0;
	bsp_io_level_t level = BSP_IO_LEVEL_LOW;
	R_IOPORT_PinRead(&g_ioport_ctrl, g_joint_limit_pins[joint_id], &level);
	return (level == BSP_IO_LEVEL_HIGH) ? 1 : 0;
}

/* External IRQ callback called by FSP when limit switch edge detected. */
void limit_sw_callback(external_irq_callback_args_t * p_args)
{
	if (p_args == NULL) return;

	/*
	 * FSP IRQ channel 与关节映射：
	 * ch1->joint1, ch2->joint2, ch3->joint3, ch4->joint6(P411), ch5->joint5, ch6->joint4(P301)
	 */
	static const uint8_t irq_channel_to_joint[ROBOT_MAX_JOINT_NUM] = {
		0, 1, 2, 5, 4, 3
	};

	uint32_t channel = p_args->channel; /* channel is 1..6 */
	if ((channel >= 1U) && (channel <= ROBOT_MAX_JOINT_NUM)) {
		robot_joint_limit_happend(irq_channel_to_joint[channel - 1U]);
	}
}

static void robot_joint_reset(uint8_t joint_id)
{
	int state;
    int reset_dir = g_robot.joints[joint_id].reset_dir;
    
	robot_joint_limit_set_input(joint_id); // 设置为输入模式，避免误触发中断
	state = robot_get_limit_status(joint_id); // 读取限位开关状态
	robot_joint_limit_set_irq(joint_id); // 恢复为中断模式

	if (state != 0) { // 如果已经触发限位，直接复位
		LOG("joint %d limit switch already happend\n", joint_id);
		Emm_V5_Reset_CurPos_To_Zero((uint8_t)(joint_id + 1u)); // 将当前位置复位为0
        return;
    }

    robot_joint_rotate_to(joint_id, reset_dir, ROBOT_RESET_DEFAULT_ANGLE, ROBOT_RESET_DEFAULT_VELOCITY, 
                                ROBOT_RESET_DEFAULT_ACCELERATION, false);
	// 持续旋转直到触发限位开关
	while (!ROBOT_STATUS_IS(g_robot.joints[joint_id].status, ROBOT_STATUS_LIMIT_HAPPENED)) {
        vTaskDelay(200); // 延时等待
	}
    vTaskDelay(ROBOT_CAN_DELAY);
    Emm_V5_Reset_CurPos_To_Zero((uint8_t)(joint_id + 1u)); // 复位当前位置
}

/* 执行硬回零：依次将各关节转动直到触发限位开关，然后将角度设置为初始值 */
static void robot_joint_hard_reset(void)
{
	LOG("robot_joint_hard_reset start now\r\n ");
    // 按指定顺序回零：5->4->2->3->1（关节下标从0开始）
    const uint8_t reset_order[] = {4, 3, 1, 2, 0};
    for (uint32_t idx = 0; idx < (sizeof(reset_order) / sizeof(reset_order[0])); idx++) {
        uint8_t i = reset_order[idx];
        LOG("Hard resetting joint %d (addr: %d)...\r\n", i, i + 1);

        // 1. 使能电机
        Emm_V5_En_Control((uint8_t)(i + 1), true, false);
        vTaskDelay(pdMS_TO_TICKS(50)); // 等待电机使能完成

        // 2. 使能限位开关检测逻辑
        ROBOT_STATUS_SET(g_robot.joints[i].status, ROBOT_STATUS_LIMIT_ENABLE);

        // 3. 执行回零动作
        robot_joint_reset(i);
        vTaskDelay(pdMS_TO_TICKS(100)); // 等待一个关节回零完成
    }

    for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
        g_robot.joints[i].current_angle = g_joints_init[i].current_angle;
    }
    robot_set_home_pose_valid();
    ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_POSE_DEGRADED);
    robot_mqtt_joints_sync();
    g_hard_reset_done = true;
}


static int robot_angle_map(float angle, float min_angle, float max_angle, float *result)
{
    if (result == NULL) {
		return 1;
	}

	// 处理边界情况，避免限位附近的微小误差被卷绕到另一圈
	if (fabsf(angle - min_angle) <= ROBOT_JOINT_ANGLE_ERROR_RANGE) {
        *result = min_angle;
        return 0;
    }
    if (fabsf(angle - max_angle) <= ROBOT_JOINT_ANGLE_ERROR_RANGE) {
        *result = max_angle;
        return 0;
    }

    float tmp_angle = angle;

	if (angle < min_angle) {
		tmp_angle += 360.0f;
    } else if (angle > max_angle) {
    	tmp_angle -= 360.0f;
    }

	// 再次处理边界情况
	if (fabsf(tmp_angle - min_angle) <= ROBOT_JOINT_ANGLE_ERROR_RANGE) {
        *result = min_angle;
        return 0;
    }
    if (fabsf(tmp_angle - max_angle) <= ROBOT_JOINT_ANGLE_ERROR_RANGE) {
        *result = max_angle;
        return 0;
    }

	if ((tmp_angle < min_angle) || (tmp_angle > max_angle)) {
        return 1;
    }

    *result = tmp_angle;
    return 0;
}

static int robot_update_current_angle_retry(uint8_t joint_id, uint8_t retry_times)
{
	for (uint8_t t = 0; t <= retry_times; t++) {
		if (robot_update_current_angle(joint_id) == 0) {
			return 0;
		}
		vTaskDelay(pdMS_TO_TICKS(20));
	}
	LOG("joint %u update current angle retry failed.\r\n", joint_id);
	return 1;
}

static int robot_update_current_angle_from_data(uint8_t joint_id, uint8_t addr, const uint8_t *rx, uint8_t dlc)
{
    if ((joint_id >= ROBOT_MAX_JOINT_NUM) || (rx == NULL)) {
        return 1;
    }

    if ((dlc < 7u) || (rx[0] != 0x36u) || (rx[dlc - 1u] != 0x6Bu) || (addr != (uint8_t)(joint_id + 1u)))
    {
        LOG("CAN RX validation failed! Joint: %u, addr: %u, Data[0]: 0x%02x\r\n", joint_id, addr, rx[0]);
        return 1;
    }

    struct joint *joint = &g_robot.joints[joint_id];
    float angle = 0.0f;
    for (int i = 5; i >= 2; i--)
    {
        angle += (float)(((uint32_t)rx[i]) << ((5 - i) << 3));
    }

    if (rx[1] == 0x01u)
    {
        angle = -angle;
    }

    if (joint->postive_direction == MOTOR_DIR_CCW)
    {
        angle = -angle;
    }

    angle = angle * 360.0f / 65536.0f / joint->reduction_ratio + g_joints_init[joint_id].current_angle;
    joint->current_angle = robot_angle_normalize(angle);
    return 0;
}

static int robot_update_all_angles(uint8_t joint_num, uint32_t *missing_mask_out, uint32_t *elapsed_ms_out)
{
    uint32_t missing_mask = 0u;
    uint32_t start_ms = HAL_GetTick();

    if (joint_num > ROBOT_MAX_JOINT_NUM) {
        joint_num = ROBOT_MAX_JOINT_NUM;
    }
    if (joint_num == 0u) {
        if (missing_mask_out != NULL) {
            *missing_mask_out = 0u;
        }
        if (elapsed_ms_out != NULL) {
            *elapsed_ms_out = 0u;
        }
        return 0;
    }

    if (!BSP_CAN_Lock(200u)) {
        for (uint8_t i = 0; i < joint_num; i++) {
            missing_mask |= (1u << i);
        }
        if (missing_mask_out != NULL) {
            *missing_mask_out = missing_mask;
        }
        if (elapsed_ms_out != NULL) {
            *elapsed_ms_out = HAL_GetTick() - start_ms;
        }
        LOG("batch angle read: CAN bus lock timeout.\r\n");
        return 1;
    }

    BSP_CAN_ClearMotorFlags();
    for (uint8_t i = 0; i < joint_num; i++) {
        Emm_V5_Read_Sys_Params((uint8_t)(i + 1u), S_CPOS);
    }

    bool all_ok = BSP_CAN_WaitAllMotors(joint_num, 6u);

    for (uint8_t i = 0; i < joint_num; i++) {
        uint8_t rx[8] = {0};
        uint8_t dlc = 0u;
        uint8_t flag = 0u;

        __disable_irq();
        flag = g_can_context.motor_rx_flag[i];
        dlc = g_can_context.motor_rx_dlc[i];
        if (dlc > 8u) {
            dlc = 8u;
        }
        for (uint8_t j = 0; j < dlc; j++) {
            rx[j] = g_can_context.motor_rx_buf[i][j];
        }
        __enable_irq();

        if (flag == 0u) {
            missing_mask |= (1u << i);
            continue;
        }

        if (robot_update_current_angle_from_data(i, (uint8_t)(i + 1u), rx, dlc) != 0) {
            missing_mask |= (1u << i);
        }
    }

    BSP_CAN_Unlock();

    if (missing_mask_out != NULL) {
        *missing_mask_out = missing_mask;
    }
    if (elapsed_ms_out != NULL) {
        *elapsed_ms_out = HAL_GetTick() - start_ms;
    }

    if ((!all_ok) || (missing_mask != 0u)) {
        LOG("batch angle read timeout/missing mask=0x%02lX elapsed=%lu ms\r\n",
            (unsigned long)missing_mask,
            (unsigned long)(HAL_GetTick() - start_ms));
        return 1;
    }

    return 0;
}

static int robot_angle_map_soft_reset(float angle, float min_angle, float max_angle, float *result)
{
	int ret = robot_angle_map(angle, min_angle, max_angle, result);
	if (ret == 0) {
		return 0;
	}

	/* soft reset场景允许小幅越界夹紧，提高重复测试稳定性 */
	const float edge_tol = 1.0f;
	if ((angle > max_angle) && (angle <= (max_angle + edge_tol))) {
		*result = max_angle;
		return 0;
	}
	if ((angle < min_angle) && (angle >= (min_angle - edge_tol))) {
		*result = min_angle;
		return 0;
	}

	return 1;
}

static bool robot_joint_is_full_turn(uint8_t joint_id)
{
    return (fabsf(g_joints_init[joint_id].min_angle) < 1e-6f) &&
           (fabsf(g_joints_init[joint_id].max_angle - 360.0f) < 1e-6f);
}

static int robot_joint_compare_angle(uint8_t joint_id, float raw_angle, float *compare_angle)
{
    if ((joint_id >= ROBOT_MAX_JOINT_NUM) || (compare_angle == NULL)) {
        return 1;
    }

    if (robot_joint_is_full_turn(joint_id)) {
        *compare_angle = raw_angle;
        return 0;
    }

    return robot_angle_map(raw_angle, g_joints_init[joint_id].min_angle,
        g_joints_init[joint_id].max_angle, compare_angle);
}

static int robot_joint_compare_error(uint8_t joint_id, float raw_angle, float ref_angle, float *err, float *compare_angle)
{
    float cur = 0.0f;

    if ((err == NULL) || (robot_joint_compare_angle(joint_id, raw_angle, &cur) != 0)) {
        return 1;
    }

    if (compare_angle != NULL) {
        *compare_angle = cur;
    }

    *err = fabsf(cur - ref_angle);
    if (robot_joint_is_full_turn(joint_id) && (*err > 180.0f)) {
        *err = 360.0f - *err;
    }

    return 0;
}

#define ROBOT_SOFT_RESET_REFINE_PASSES            (3U)
#define ROBOT_SOFT_RESET_REFINE_ROUNDS_PER_JOINT  (3U)
#define ROBOT_SOFT_RESET_REFINE_TOL_DEG           (1.0f)
#define ROBOT_SOFT_RESET_REFINE_SETTLE_LOOPS      (10)
#define ROBOT_SOFT_RESET_REFINE_VELOCITY          (4.0f)

static bool robot_soft_reset_refine_joint(uint8_t joint_id, uint8_t rounds, float tol_deg)
{
    float target = g_joints_init[joint_id].current_angle;

    for (uint8_t round = 0; round < rounds; round++) {
        if (robot_update_current_angle_retry(joint_id, 2u) != 0) {
            continue;
        }

        float cur = 0.0f;
        float err = 0.0f;
        if (robot_joint_compare_error(joint_id, g_robot.joints[joint_id].current_angle,
                target, &err, &cur) != 0) {
            continue;
        }

        g_robot.joints[joint_id].current_angle = cur;
        if (err <= tol_deg) {
            return true;
        }

        float signed_err = robot_angle_diff(cur, target);
        int dir = (signed_err >= 0.0f) ? DIR_POSITIVE : DIR_NEGATIVE;

        LOG("soft reset refine joint %u round %u: cur=%.2f target=%.2f err=%.2f dir=%d\r\n",
            (unsigned)joint_id, (unsigned)(round + 1u), cur, target, err, dir);

        if (robot_joint_rotate_to(joint_id, dir, target,
                ROBOT_SOFT_RESET_REFINE_VELOCITY,
                (ROBOT_RESET_DEFAULT_ACCELERATION / 2U), true) != 0) {
            continue;
        }

        int stable_hit = 0;
        for (int k = 0; k < ROBOT_SOFT_RESET_REFINE_SETTLE_LOOPS; k++) {
            vTaskDelay(pdMS_TO_TICKS(40));
            if (robot_update_current_angle_retry(joint_id, 1u) != 0) {
                stable_hit = 0;
                continue;
            }

            float cur2 = 0.0f;
            float err2 = 0.0f;
            if (robot_joint_compare_error(joint_id, g_robot.joints[joint_id].current_angle,
                    target, &err2, &cur2) != 0) {
                stable_hit = 0;
                continue;
            }

            g_robot.joints[joint_id].current_angle = cur2;
            if (err2 <= tol_deg) {
                stable_hit++;
                if (stable_hit >= 2) {
                    return true;
                }
            } else {
                stable_hit = 0;
            }
        }
    }

    return false;
}

static void robot_joint_soft_reset(void)
{
	LOG("--- Starting soft reset ---\r\n");

    /* 1. 仅使能参与机械臂复位的关节，J6(关节下标5)由夹爪模块单独管理 */
    const uint8_t reset_order[] = {4, 3, 1, 2, 0};
    for (uint32_t idx = 0; idx < (sizeof(reset_order) / sizeof(reset_order[0])); idx++) {
        uint8_t i = reset_order[idx];
        Emm_V5_En_Control((uint8_t)(i + 1), true, false);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    LOG("Soft reset motors enabled (J1~J5, J6 skipped).\r\n");

	float angle = 0;
	int ret = 0;
	int fail_count = 0;
	for (uint32_t idx = 0; idx < (sizeof(reset_order) / sizeof(reset_order[0])); idx++)
	{
        uint8_t i = reset_order[idx];
        LOG("Soft resetting joint %d...\r\n", i);

		ROBOT_STATUS_SET(g_robot.joints[i].status, ROBOT_STATUS_LIMIT_ENABLE);
		int dir = DIR_POSITIVE;

		ret = robot_update_current_angle_retry((uint8_t)i, 2);
		if (ret != 0) {
			LOG("soft reset skip joint %d: read angle failed\r\n", i);
			fail_count++;
			continue;
		}
		LOG("robot_update_current_angle %d...\r\n", i);

		ret = robot_angle_map_soft_reset(g_robot.joints[i].current_angle, g_joints_init[i].min_angle, g_joints_init[i].max_angle, &angle);
		if (ret != 0) {
			LOG("soft reset skip joint %d: angle map failed, current:%.2f range:[%.2f, %.2f]\r\n", i,
				g_robot.joints[i].current_angle, g_joints_init[i].min_angle, g_joints_init[i].max_angle);
			fail_count++;
			continue;
		}

		if (angle > g_joints_init[i].current_angle) { // 需要反向旋转
			dir = DIR_NEGATIVE;
		}

		if (fabsf(g_joints_init[i].min_angle) < 1e-6f && fabsf(g_joints_init[i].max_angle - 360.0f) < 1e-6f) {
			if (fabsf(angle - g_joints_init[i].current_angle) > 180.0f) { // 选择最短路径
				dir = -dir;
			}
		}

		LOG("[%d] current:%.2f, target:%.2f, dir:%d\r\n", i, angle, g_joints_init[i].current_angle, dir);
		g_robot.joints[i].current_angle = angle;

		ret = (int)robot_joint_rotate_to((uint32_t)i, dir, g_joints_init[i].current_angle,
			ROBOT_RESET_DEFAULT_VELOCITY, ROBOT_RESET_DEFAULT_ACCELERATION, true);
		if (ret != 0) {
			LOG("soft reset joint %d rotate command failed\r\n", i);
			fail_count++;
			continue;
		}

		/* 自适应等待：按偏差角度估算行进时间 + 300ms 余量，每 40ms 轮询一次
		 * ROBOT_RESET_DEFAULT_VELOCITY(rpm) * 6.0 = deg/s */
		float err_deg_init = fabsf(g_joints_init[i].current_angle - angle);
		float travel_ms = err_deg_init * 1000.0f / (ROBOT_RESET_DEFAULT_VELOCITY * 6.0f);
		int max_settle_loops = (int)((travel_ms + 300.0f) / 40.0f);
		if (max_settle_loops < 8)  max_settle_loops = 8;   /* 最少 320ms */
		if (max_settle_loops > 75) max_settle_loops = 75;  /* 最多 3s */

		int settle_ok = 0;
		int stable_hit = 0;
		for (int k = 0; k < max_settle_loops; k++) {
			vTaskDelay(pdMS_TO_TICKS(40));
			if (robot_update_current_angle_retry((uint8_t)i, 1) != 0) {
				stable_hit = 0;
				continue;
			}

			float cur = 0.0f;
			float err = 0.0f;
			if (robot_joint_compare_error((uint8_t)i, g_robot.joints[i].current_angle,
					g_joints_init[i].current_angle, &err, &cur) != 0) {
				stable_hit = 0;
				continue;
			}

			g_robot.joints[i].current_angle = cur;
			if (err <= 1.0f) {
				stable_hit++;
				if (stable_hit >= 2) {
					settle_ok = 1;
					break;
				}
			} else {
				stable_hit = 0;
			}
		}


		if (!settle_ok) {
			LOG("soft reset joint %d settle timeout, keep going\r\n", i);
			fail_count++;
		}
	}

    int bad_joint = -1;
    float bad_err = 0.0f;
    bool home_ok = robot_verify_home_pose(2u, 1.0f, &bad_joint, &bad_err);

    if (!home_ok) {
        LOG("soft reset verify not pass, start refine loop...\r\n");
        for (uint8_t pass = 0; pass < ROBOT_SOFT_RESET_REFINE_PASSES; pass++) {
            for (uint32_t idx = 0; idx < (sizeof(reset_order) / sizeof(reset_order[0])); idx++) {
                uint8_t i = reset_order[idx];
                (void)robot_soft_reset_refine_joint(i, ROBOT_SOFT_RESET_REFINE_ROUNDS_PER_JOINT,
                    ROBOT_SOFT_RESET_REFINE_TOL_DEG);
            }

            bad_joint = -1;
            bad_err = 0.0f;
            home_ok = robot_verify_home_pose(2u, ROBOT_SOFT_RESET_REFINE_TOL_DEG, &bad_joint, &bad_err);
            if (home_ok) {
                LOG("soft reset refine pass %u success.\r\n", (unsigned)(pass + 1u));
                break;
            }

            LOG("soft reset refine pass %u not enough, continue...\r\n", (unsigned)(pass + 1u));
        }
    }

    if (home_ok) {
        robot_set_home_pose_valid();
        ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_POSE_DEGRADED);
        LOG("soft reset final verify PASS (issues:%d), pose set HOME.\r\n", fail_count);
    } else {
        ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_POSE_VALID);
        ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_POSE_DEGRADED);

        if (bad_err < -1.5f) {
            LOG("soft reset final FAIL: joint %d angle map failed after refine, pose invalid.\r\n", bad_joint);
        } else if (bad_err < -0.5f) {
            LOG("soft reset final FAIL: joint %d read angle failed after refine, pose invalid.\r\n", bad_joint);
        } else {
            LOG("soft reset final FAIL: joint %d err=%.2fdeg after refine, pose invalid.\r\n", bad_joint, bad_err);
        }
    }

    robot_mqtt_joints_sync();
    g_soft_reset_done = true;
    /* 复位完成，清除 IK 分支锁，避免复位后锁定到旧分支 */
    robot_kinematics_reset_branch_lock();
}


static struct position *robot_time_func_path_interpolation(uint32_t time_limit_ms, int *size)

{
    int path_size = 0;
	struct position pos = {0};
	int ret = 0;

    if (g_robot_time_func == NULL) {
        LOG("robot time func is null\n");
		return NULL;	
    }

	path_size = (int)(time_limit_ms / (uint32_t)ROBOT_INTERPOLATION_TIME_RESOLUTION);
    if (path_size > ROBOT_MAX_PATH_SIZE) {
        LOG("[WARN] time_func path_size %d > ROBOT_MAX_PATH_SIZE %d, truncated\r\n",
            path_size, ROBOT_MAX_PATH_SIZE);
        path_size = ROBOT_MAX_PATH_SIZE;
    }
    struct position *path = s_path_buf;

    for (int i = 0; i < path_size; i++) {
        ret = g_robot_time_func((uint32_t)i * (uint32_t)ROBOT_INTERPOLATION_TIME_RESOLUTION, &pos);
		if (ret != 0) {
			LOG("robot time func failed\n");
			return NULL;
		}

        path[i].x = pos.x;
        path[i].y = pos.y;
        path[i].z = pos.z;
    }

    *size = path_size;
    return path;
}

static void robot_time_func_move(uint32_t time_limit_ms)
{
	int ret;
	int path_size = 0;

	// 生成时间函数插补轨迹（使用静态缓冲区）
	struct position *path = robot_time_func_path_interpolation(time_limit_ms, &path_size);
    if (path == NULL) {
        LOG("robot time func failed\n");
        return;
    }

	// 逆解结果使用静态缓冲区
    float *result = s_result_buf;

	// 更新当前关节角度到运动学模块
    for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
        robot_kinematics_joint_angle_update_by_id((uint32_t)i, g_robot.joints[i].current_angle);
    }
    /* 新运动起步：复位 IK 分支锁，首帧从实际姿态最近解开始 */
    robot_kinematics_reset_branch_lock();

    /* time_func 过程中锁定 J6，避免夹爪随整臂轨迹抖动 */
    float j6_hold_angle = g_robot.joints[ROBOT_JOINT_6].current_angle;

	for (int i = 0; i < path_size; i++) {
        robot_kinematics_cal_T(T_0_6_reset, g_robot.T, &path[i]);
        ret = robot_kinematics_inverse((float *)g_robot.T, &result[i * ROBOT_MAX_JOINT_NUM], false);

        if (ret != 0) {
            LOG("robot_time_func_move robot kinematics inverse failed\n");
            return;
        }

        result[i * ROBOT_MAX_JOINT_NUM + ROBOT_JOINT_6] = j6_hold_angle;

        // --- 数据合法性检查 ---

        for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
            float angle = result[i * ROBOT_MAX_JOINT_NUM + j];
            if (isnan(angle) || isinf(angle) || fabsf(angle) > 1000.0f) {
                LOG("ERROR: IK failed at point %d, joint %d. Invalid angle calculated: %.2f\r\n", i, j, angle);
                LOG("Aborting time func move.\r\n");
                return;
            }
        }

		// 将逆解结果更新到运动学模块，用于下一次计算
		robot_kinematics_joint_angle_update(&result[i * ROBOT_MAX_JOINT_NUM]);
		LOG("[%d] <%.2f %.2f %.2f> ", i, path[i].x, path[i].y, path[i].z);
		LOG("result: ");
		for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
			LOG("%.2f ", result[i * ROBOT_MAX_JOINT_NUM + j]);
		}
		LOG("\n");
    }

	ret = robot_pid_run(path, path_size, result);
	if (ret == 0) {
		g_robot.cur_pos.x = path[path_size -1].x;
		g_robot.cur_pos.y = path[path_size -1].y;
		g_robot.cur_pos.z = path[path_size -1].z;
        robot_try_refresh_joints_feedback(1u);
	}
}

static void robot_auto_move_interpolation(struct robot_event *event)
{
    int ret;
	// 生成直线插补路径（使用静态缓冲区，消除运行时 malloc）
	int path_size = 0;
	struct position *target_pos = (struct position*)event->param;
    struct position *path   = s_path_buf;
    float           *result = s_result_buf;

    /* 生成 S 曲线路径（写入 s_path_buf），返回点数已受 ROBOT_MAX_PATH_SIZE 限制 */
    if (robot_path_interpolation_scurve(target_pos, &path_size) <= 0) {
        LOG("[WARN] scurve path generation failed\r\n");
        return;
    }

	// 更新当前关节角度到运动学模块
    for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
        robot_kinematics_joint_angle_update_by_id((uint32_t)i, g_robot.joints[i].current_angle);
    }
    /* 新运动起步：复位 IK 分支锁，首帧从实际姿态最近解开始 */
    robot_kinematics_reset_branch_lock();

    /* auto 过程中锁定 J6，避免夹爪随整臂轨迹抖动 */
    float j6_hold_angle = g_robot.joints[ROBOT_JOINT_6].current_angle;

    for (int i = 0; i < path_size; i++) {
        robot_kinematics_cal_T(T_0_6_reset, g_robot.T, &path[i]);
        ret = robot_kinematics_inverse((float *)g_robot.T, &result[i * ROBOT_MAX_JOINT_NUM], false);

        if (ret != 0) {
            LOG("robot_auto_move_interpolation robot kinematics inverse failed\n");
            return;
        }

        result[i * ROBOT_MAX_JOINT_NUM + ROBOT_JOINT_6] = j6_hold_angle;

		// 将逆解角度更新到运动学模块

		robot_kinematics_joint_angle_update(&result[i * ROBOT_MAX_JOINT_NUM]);
		LOG("[%d] <%.2f %.2f %.2f> ", i, path[i].x, path[i].y, path[i].z);
		LOG("result: ");
		for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
			LOG("%.2f ", result[i * ROBOT_MAX_JOINT_NUM + j]);
		}
		LOG("\n");
    }

	ret = robot_pid_run(path, path_size, result);
	if (ret == 0) {
		g_robot.cur_pos.x = target_pos->x;
		g_robot.cur_pos.y = target_pos->y;
		g_robot.cur_pos.z = target_pos->z;
        robot_try_refresh_joints_feedback(1u);
	}
}

static float robot_angle_normalize(float angle)
{
	// 将角度规范化到 0-360 范围
	if (angle >= 360) {
		return angle - 360;	
	}

	if (angle < 0) {
		return angle + 360;	
	}
	return angle;
}

/**
 * @brief 计算两个角度之间的最短差值
 * 
 * 处理角度环绕问题，确保返回 -180 到 180 之间的差值
 * 例如，从 10° 到 350° 的最短路径是 -20° 而不是 +340°
 * 
 * @param cur_angle 当前角度
 * @param target_angle 目标角度
 * @return float 角度差，范围在 -180 到 180 之间
 */
static float robot_angle_diff(float cur_angle, float target_angle)
{
	float diff = target_angle - cur_angle;
	if (diff > 180) {
		diff -= 360;
	} else if (diff < -180) {
		diff += 360;
	}
	return diff;
}

/* 各关节独立比例系数（单位: 1/s）
 * J2(Kp=4.0): 减速比99.99，已调好
 * J3(Kp=2.5): 承载前臂重量，稳态滞后1.7°，上调至2.5 */
static const float ROBOT_JOINT_KP[ROBOT_MAX_JOINT_NUM] = {0.65f, 4.0f, 2.50f, 1.00f, 1.00f, 10.0f};

static int robot_pid_run(struct position *path, int path_size, float *result)
{
	(void)path;
	int p;
	float target_angle[ROBOT_MAX_JOINT_NUM] = {0};
	float feedforward[ROBOT_MAX_JOINT_NUM] = {0};
	float total_error[ROBOT_MAX_JOINT_NUM] = {0};
	int sample_count = 0;

	for (p = 1; p < path_size; p++) {
		for (int j = 0; j < ROBOT_ARM_JOINT_NUM; j++) {
			float angle = result[p * ROBOT_MAX_JOINT_NUM + j];
			if (isnan(angle) || isinf(angle) || fabsf(angle) > 10000.0f) {
				LOG("ERROR: PID run received invalid angle for point %d, joint %d. Value: %.2f\r\n", p, j, angle);
				LOG("Aborting PID run.\r\n");
				robot_joint_stop_all(ROBOT_MAX_JOINT_NUM);
				return 1;
			}
			target_angle[j] = robot_angle_normalize(angle);
			/* 节点前馈：S 曲线保证每10ms步进一次目标，全量前馈无超调风险 */
			float prev_angle = robot_angle_normalize(result[(p - 1) * ROBOT_MAX_JOINT_NUM + j]);
			feedforward[j] = robot_angle_diff(prev_angle, target_angle[j])
			                  / (ROBOT_INTERPOLATION_TIME_RESOLUTION / 1000.0f);
		}

		/* 每个路径点恰好执行一个控制周期(one_period 自带 10ms 节拍)，
		 * 不再用外层 while 二次定时，避免跳点/重复导致的冲击与抖动 */
		robot_pid_one_period(target_angle, feedforward, total_error, ROBOT_ARM_JOINT_NUM);
		sample_count++;

		if ((p % 10) == 0) {
			robot_mqtt_joints_sync();
		}
	}

	/* 末端稳定段：保持终点目标、前馈清零，让 P 控制器平滑收敛后再停止，消除末端冲击 */
	for (int j = 0; j < ROBOT_ARM_JOINT_NUM; j++) {
		feedforward[j] = 0.0f;
	}
	for (int k = 0; k < ROBOT_PID_SETTLE_PERIODS; k++) {
		robot_pid_one_period(target_angle, feedforward, total_error, ROBOT_ARM_JOINT_NUM);
		sample_count++;
	}

	robot_joint_stop_all(ROBOT_ARM_JOINT_NUM);
	if (sample_count > 0) {
		for (int j = 0; j < ROBOT_ARM_JOINT_NUM; j++) {
			LOG("[jpint %d] ave_error:%.2f\n", j + 1, total_error[j] / (float)sample_count);
		}
	}

	LOG("\nrobot pid run finished!!\n");
	return 0;
}

static void robot_joints_sync_to(struct robot_event *event)
{
	for (int i = 0; i < ROBOT_ARM_JOINT_NUM; i++) {
		robot_joint_rotate_to((uint32_t)i, DIR_POSITIVE, event->param[i],
					ROBOT_JOINT_DEFAULT_VELOCITY, ROBOT_JOINT_DEFAULT_ACCELERATION, true);
		g_robot.joints[i].current_angle = event->param[i]; // 更新当前关节角度
	}
}


/**
 * @brief 机器人控制任务
 * 
 * 该任务负责处理各种机器人事件，包括关节旋转、限位处理、自动运动、
 * 时间函数运动、重置操作、远程控制等
 * 
 * @param arg 未使用的参数
 */
static void robot_control_task(void *arg)
{
	(void)arg;
    LOG("robot control task runing!!!\n");
    
	struct robot_event event = {0};
	// 无限循环等待事件
    while(xQueueReceive(g_robot.event_queue, &event, portMAX_DELAY) == pdPASS) {
        switch (event.type) {
            case ROBOT_JOINT_REL_ROTATE:
				LOG("[joint_id: %d] ROBOT_JOINT_REL_ROTATE %f\n", event.joint_id, event.param[0]);
                robot_joint_rotate_to((uint32_t)event.joint_id, DIR_POSITIVE,event.param[0],
						ROBOT_JOINT_DEFAULT_VELOCITY, ROBOT_JOINT_DEFAULT_ACCELERATION, false);
                break; 
			case ROBOT_JOINT_ABS_ROTATE:
				LOG("[joint_id: %d] ROBOT_JOINT_ABS_ROTATE %f\n", event.joint_id, event.param[0]);
				robot_joint_rotate_to((uint32_t)event.joint_id, DIR_POSITIVE, event.param[0],
					ROBOT_JOINT_DEFAULT_VELOCITY, ROBOT_JOINT_DEFAULT_ACCELERATION, true);
				break;
			case ROBOT_LIMIT_SWITCH_EVENT:
				LOG("[joint_id: %d] ROBOT_LIMIT_SWITCH_EVENT\n", event.joint_id);
				robot_joint_limit_post_handle(event.joint_id);
				break;
			case ROBOT_AUTO_EVENT:
				LOG("ROBOT_AUTO_EVENT\n");
				robot_auto_move_interpolation(&event);
				break;
			case ROBOT_TIME_FUNC_EVENT:
				LOG("ROBOT_TIME_FUNC_EVENT\n");
				robot_time_func_move((uint32_t)(event.param[0]));
				break;
			case ROBOT_HARD_RESET_EVENT:
				LOG("ROBOT_HARD_RESET_EVENT\n");
				robot_joint_hard_reset();
				break;
			case ROBOT_SOFT_RESET_EVENT:
				LOG("ROBOT_SOFT_RESET_EVENT\n");
				robot_joint_soft_reset();
				break;
			case ROBOT_TEST_EVENT:
				LOG("ROBOT_RESET_EVENT\n");
				robot_update_current_angle(event.joint_id);
				break;
			case ROBOT_REMOTE_CONTROL_EVENT:
				LOG("ROBOT_REMOTE_CONTROL_EVENT\n");
				robot_pid_remote();
				break;
			case ROBOT_JOINTS_SYNC_EVENT:
				LOG("ROBOT_JOINTS_SYNC_EVENT\n");
				robot_joints_sync_to(&event);
				break;
            case ROBOT_READ_ALL_EVENT:
                LOG("ROBOT_READ_ALL_EVENT\n");
                robot_read_all_debug();
                break;
			default:
				LOG("robot event type error\n");
        }
    }
}

static void robot_read_all_debug(void)
{
    uint32_t missing_mask = 0u;
    uint32_t elapsed_ms = 0u;
    int ret = robot_update_all_angles(ROBOT_ARM_JOINT_NUM, &missing_mask, &elapsed_ms);

    LOG("read_all ret=%d elapsed=%lu ms missing=0x%02lX\r\n",
        ret,
        (unsigned long)elapsed_ms,
        (unsigned long)missing_mask);
    for (uint8_t i = 0; i < ROBOT_ARM_JOINT_NUM; i++) {
        LOG("J%u angle=%.2f\r\n", (unsigned int)(i + 1u), g_robot.joints[i].current_angle);
    }
}

static void robot_pid_one_period(float *target_angle, float *feedforward, float *total_error, int joint_num)
{
	uint32_t pid_end_time = xTaskGetTickCount() + ROBOT_PID_PERIOD;
	(void)robot_update_all_angles((uint8_t)joint_num, NULL, NULL);
	for (int j = 0; j < joint_num; j++) {
		float error = robot_angle_diff(g_robot.joints[j].current_angle, target_angle[j]);
		if (total_error != NULL) {
			total_error[j] += fabsf(error);
		}
		float v = feedforward[j] + ROBOT_JOINT_KP[j] * error;
		if (v >  ROBOT_FF_OUTPUT_LIMIT) v =  ROBOT_FF_OUTPUT_LIMIT;
		if (v < -ROBOT_FF_OUTPUT_LIMIT) v = -ROBOT_FF_OUTPUT_LIMIT;
		robot_joint_velocity_nowait((uint32_t)j, v, ROBOT_JOINT_DEFAULT_ACCELERATION);
	}
	uint32_t now = xTaskGetTickCount();
	if (now < pid_end_time) {
		vTaskDelay(pid_end_time - now);
	}
}

struct position g_pos = {0};	// debug
static int robot_pid_remote(void)
{
	uint64_t end_time = 0;
	float target_angle[ROBOT_MAX_JOINT_NUM] = {0};
	float last_target[ROBOT_MAX_JOINT_NUM] = {0};
	float feedforward[ROBOT_MAX_JOINT_NUM] = {0};
	bool first_update = true;
	int ret;
	float T[4][4] = {0};
	int error_count = 0;

	LOG("wait robot reset....\n");
	vTaskDelay(3000);
	LOG("robot into remote mode!!!!\n");

	/* 进入遥控连续控制前复位 IK 分支锁 */
	robot_kinematics_reset_branch_lock();

	end_time = xTaskGetTickCount();
	while(ROBOT_STATUS_IS(g_robot.status, ROBOT_STATUS_RMODE_ENABLE)) {
		end_time += (uint32_t)ROBOT_REMOTE_TIME_RESOLUTION; // 增加时间步长, 控制循环频率

		// 计算新位置
		g_pos.x = g_robot.cur_pos.x + g_remote_control.vx * (float)ROBOT_REMOTE_TIME_RESOLUTION / 1000.0f;
		g_pos.y = g_robot.cur_pos.y + g_remote_control.vy * (float)ROBOT_REMOTE_TIME_RESOLUTION / 1000.0f;
		g_pos.z = g_robot.cur_pos.z + g_remote_control.vz * (float)ROBOT_REMOTE_TIME_RESOLUTION / 1000.0f;
		robot_kinematics_cal_T(T_0_6_reset, T, &g_pos);
		ret = robot_kinematics_inverse((float *)T, &g_remote_control.result[0], false);
		if (ret < 0) {
			error_count++;
			if (error_count >= 10) {
				LOG("robot_pid_remote robot kinematics inverse failed\n");
				error_count = 0;
			}
			
			robot_joint_stop_all(ROBOT_MAX_JOINT_NUM);
			continue;
		}
		error_count = 0;
		robot_kinematics_joint_angle_update(&g_remote_control.result[0]);
		g_robot.cur_pos.x = g_pos.x;
		g_robot.cur_pos.y = g_pos.y;
		g_robot.cur_pos.z = g_pos.z;
		robot_joint_veloccity_to(4u, g_remote_control.rx, ROBOT_JOINT_DEFAULT_ACCELERATION);
		/* J6 由夹爪模块独立控制，remote 不再驱动关节6 */


		for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
			target_angle[j] = robot_angle_normalize(g_remote_control.result[j]);
			/* remote 前馈：相邻目标角差 / 时间步长；首次更新为 0，避免进入时冲击 */
			if (first_update) {
				feedforward[j] = 0.0f;
			} else {
				feedforward[j] = robot_angle_diff(last_target[j], target_angle[j])
				                  / (ROBOT_REMOTE_TIME_RESOLUTION / 1000.0f);
			}
			last_target[j] = target_angle[j];
		}
		first_update = false;

		while(xTaskGetTickCount() < end_time) {
			robot_pid_one_period(target_angle, feedforward, NULL, 4);
		}
	}

	robot_joint_stop_all(ROBOT_MAX_JOINT_NUM);

	LOG("\nrobot remote disable!!\n");
	return 0;
}

/**
 * @brief 机器人系统初始化
 * 
 * 初始化机器人关节参数、创建事件队列、创建控制任务和命令服务任务
 */
void robot_init(void)
{
    /* 打开外部中断(限位开关) */
    if (JOINT_LIMIT_1.p_api->open(JOINT_LIMIT_1.p_ctrl, JOINT_LIMIT_1.p_cfg) != FSP_SUCCESS) {
        LOG("JOINT_LIMIT_1 open failed\r\n");
    }
    if (JOINT_LIMIT_2.p_api->open(JOINT_LIMIT_2.p_ctrl, JOINT_LIMIT_2.p_cfg) != FSP_SUCCESS) {
        LOG("JOINT_LIMIT_2 open failed\r\n");
    }
    if (JOINT_LIMIT_3.p_api->open(JOINT_LIMIT_3.p_ctrl, JOINT_LIMIT_3.p_cfg) != FSP_SUCCESS) {
        LOG("JOINT_LIMIT_3 open failed\r\n");
    }
    if (JOINT_LIMIT_4.p_api->open(JOINT_LIMIT_4.p_ctrl, JOINT_LIMIT_4.p_cfg) != FSP_SUCCESS) {
        LOG("JOINT_LIMIT_4 open failed\r\n");
    }
    if (JOINT_LIMIT_5.p_api->open(JOINT_LIMIT_5.p_ctrl, JOINT_LIMIT_5.p_cfg) != FSP_SUCCESS) {
        LOG("JOINT_LIMIT_5 open failed\r\n");
    }
    if (JOINT_LIMIT_6.p_api->open(JOINT_LIMIT_6.p_ctrl, JOINT_LIMIT_6.p_cfg) != FSP_SUCCESS) {
        LOG("JOINT_LIMIT_6 open failed\r\n");
    }

    /* 初始化关节参数 */
    memcpy(g_robot.joints, g_joints_init, sizeof(g_joints_init));
    memcpy(g_robot.T, T_0_6_reset, sizeof(T_0_6_reset));
    ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_POSE_VALID);
    ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_POSE_DEGRADED);

	g_robot.event_queue = xQueueCreate(ROBOT_MAX_EVENT_NUM, sizeof(struct robot_event));
    if (g_robot.event_queue == NULL) {
      	LOG("create robot event queue failed\n");
      	return; 
  	}

    g_robot.cmd_queue = xQueueCreate(ROBOT_CMD_MAX_NUM, sizeof(struct robot_cmd));
    if (g_robot.cmd_queue == NULL) {
    	LOG("create robot cmd queue failed\n");
    	return;
    }

	/* Create robot control task using FreeRTOS xTaskCreate */
	BaseType_t xr;
	xr = xTaskCreate(robot_control_task, "robot_control_task", ROBOT_CONTROL_TASK_STACK_SIZE/4, NULL, ROBOT_CONTROL_TASK_PRIORITY, &g_robot.control_handle);
	if (pdPASS != xr) {
		LOG("create robot control task failed\n");
		return;
	}

	/* Create robot cmd service task */
	xr = xTaskCreate(robot_cmd_service, "robot_cmd_service", ROBOT_CMD_SERVICE_STACK_SIZE/4, NULL, ROBOT_CMD_SERVICE_PRIORITY, &g_robot.cmd_service_handle);
	if (pdPASS != xr) {
		LOG("create robot cmd service task failed\n");
		return;
	}
}

int robot_send_joints_sync_event(float *angles)
{
	if (g_robot.event_queue == NULL) {
		return -1;	
	}

	struct robot_event event = {0};
	event.type = ROBOT_JOINTS_SYNC_EVENT;
	memcpy(event.param, angles, sizeof(float) * ROBOT_MAX_JOINT_NUM);
	return (int)xQueueSendToBack(g_robot.event_queue, &event, ROBOT_CMD_QUEUE_TIMEOUT);
}

int robot_send_rel_rotate_event(uint8_t joint_id, float angle)
{
	struct robot_event event = {0};
	event.type = ROBOT_JOINT_REL_ROTATE;
	event.joint_id = joint_id;
	event.param[0] = angle;
    return (int)xQueueSendToBack(g_robot.event_queue, &event, ROBOT_CMD_QUEUE_TIMEOUT);
}

int robot_send_remote_event(void)
{
	struct robot_event event = {0};
	event.type = ROBOT_REMOTE_CONTROL_EVENT;
	return (int)xQueueSendToBack(g_robot.event_queue, &event, ROBOT_CMD_QUEUE_TIMEOUT);
}

int robot_send_abs_rotate_event(uint8_t joint_id, float angle)
{
	struct robot_event event = {0};
	event.type = ROBOT_JOINT_ABS_ROTATE;
	event.joint_id = joint_id;
	event.param[0] = angle;
    return (int)xQueueSendToBack(g_robot.event_queue, &event, ROBOT_CMD_QUEUE_TIMEOUT);
}

int robot_send_auto_event(struct position *pos)
{
    if ((pos == NULL) || (g_robot.event_queue == NULL)) {
        return -1;
    }

    if (!ROBOT_STATUS_IS(g_robot.status, ROBOT_STATUS_POSE_VALID)) {
        LOG("reject AUTO: pose invalid, please do hard_reset/soft_reset first.\r\n");
        return -1;
    }

    if (ROBOT_STATUS_IS(g_robot.status, ROBOT_STATUS_POSE_DEGRADED)) {
        LOG("AUTO in degraded mode: continue with estimated pose + online compensation.\r\n");
    }

	struct robot_event event = {0};
	event.type = ROBOT_AUTO_EVENT;
	memcpy(event.param, pos, sizeof(struct position));
	return (int)xQueueSendToBack(g_robot.event_queue, &event, ROBOT_CMD_QUEUE_TIMEOUT);
};

int robot_send_time_func_event(float time_limit_ms, float radius_mm)
{
	if (g_robot.event_queue == NULL) {
		return -1;
	}

	if (!ROBOT_STATUS_IS(g_robot.status, ROBOT_STATUS_POSE_VALID)) {
		LOG("reject time_func: pose invalid, please do hard_reset/soft_reset first.\r\n");
		return -1;
	}

	if (time_limit_ms <= 0.0f) {
		LOG("reject time_func: invalid time %.1f ms.\r\n", time_limit_ms);
		return -1;
	}

	if (radius_mm <= 0.0f) {
		radius_mm = 15.0f;
	}
	if (radius_mm > ROBOT_CIRCLE_RADIUS_MAX_MM) {
		LOG("circle radius %.1f mm too large, clamp to %.1f mm.\r\n",
			radius_mm, ROBOT_CIRCLE_RADIUS_MAX_MM);
		radius_mm = ROBOT_CIRCLE_RADIUS_MAX_MM;
	}
	g_time_func_circle_radius_mm = radius_mm;

	struct robot_event event = {0};
	event.type = ROBOT_TIME_FUNC_EVENT;
	event.param[0] = time_limit_ms;
	return (int)xQueueSendToBack(g_robot.event_queue, &event, ROBOT_CMD_QUEUE_TIMEOUT);
}

int robot_send_read_all_event(void)
{
    if (g_robot.event_queue == NULL) {
        return -1;
    }

    struct robot_event event = {0};
    event.type = ROBOT_READ_ALL_EVENT;
    return (int)xQueueSendToBack(g_robot.event_queue, &event, ROBOT_CMD_QUEUE_TIMEOUT);
}

int robot_send_reset_event(bool hard_reset)
{
	struct robot_event event = {0};
	if (hard_reset) {
		event.type = ROBOT_HARD_RESET_EVENT;
        g_hard_reset_done = false;
	} else {
		event.type = ROBOT_SOFT_RESET_EVENT;
        g_soft_reset_done = false;
	}
	return (int)xQueueSendToBack(g_robot.event_queue, &event, ROBOT_CMD_QUEUE_TIMEOUT);
}

bool robot_is_soft_reset_done(void)
{
    return g_soft_reset_done;
}

bool robot_is_hard_reset_done(void)
{
    return g_hard_reset_done;
}


void robot_cmd_send_from_isr(char *cmd, enum cmd_type type)
{
	if (g_robot.cmd_queue == NULL) {
		return;	
	}

	struct robot_cmd robot_cmd = {0};
	robot_cmd.type = type;
	size_t len = strlen(cmd);
	if (len >= ROBOT_CMD_LENGTH) {
		len = ROBOT_CMD_LENGTH - 1u;
	}
	strncpy(robot_cmd.cmd, cmd, len);
	robot_cmd.cmd[len] = '\0';
	
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendToBackFromISR(g_robot.cmd_queue, &robot_cmd, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief 将指令字符串发送到命令队列 (任务安全)
 * 
 * @param cmd 要发送的指令字符串
 * @param type 指令来源 (e.g., UART, MQTT)
 * @return int 0 表示成功, -1 表示失败
 */
int robot_cmd_send(char *cmd, enum cmd_type type)
{
    if (g_robot.cmd_queue == NULL) {
        return -1;
    }

    struct robot_cmd robot_cmd = {0};
    robot_cmd.type = type;

    // 安全地复制指令字符串
    size_t len = strlen(cmd);
    if (len >= ROBOT_CMD_LENGTH) {
        len = ROBOT_CMD_LENGTH - 1;
    }
    strncpy(robot_cmd.cmd, cmd, len);
    robot_cmd.cmd[len] = '\0'; // 确保字符串正确结尾

    if (xQueueSendToBack(g_robot.cmd_queue, &robot_cmd, pdMS_TO_TICKS(ROBOT_CMD_QUEUE_TIMEOUT)) != pdPASS) {
        return -1; // 发送到队列失败
    }

    return 0; // 成功
}

/* External IRQ handling is performed by limit_sw_callback (FSP external IRQ). */

static int robot_mqtt_joints_sync(void)
{
#if defined(ROBOT_MQTT_ENABLE) && (ROBOT_MQTT_ENABLE == 1)
	char msg[256] = {0};
	snprintf(msg, sizeof(msg), "[PC][%d][%.2f %.2f %.2f %.2f %.2f %.2f]", ROBOT_JOINTS_SYNC_EVENT,
				g_robot.joints[0].current_angle, g_robot.joints[1].current_angle,
				g_robot.joints[2].current_angle, g_robot.joints[3].current_angle,
				g_robot.joints[4].current_angle, g_robot.joints[5].current_angle);
	return w800_publish_message(MQTT_TOPIC, msg, 0);
#else
	return 0;
#endif
}

void robot_cmd_service(void *pvParameters)
{
	(void)pvParameters;
	struct robot_cmd rb_cmd = {0};

#if defined(ROBOT_MQTT_ENABLE) &&  (ROBOT_MQTT_ENABLE == 1)
	int ret;
	LOG("robot mqtt service init...    \n");
	vTaskDelay(pdMS_TO_TICKS(1000));    // 等待 w800 初始化
    ret = w800_mqtt_service_init();
	if (!ret) {
		LOG("robot mqtt service init failed, continuing to UART mode...\n");
	} else {
	    LOG("robot mqtt service init successed\n");
    }
#endif

    LOG("robot cmd service task ready and running!\n");

	while(xQueueReceive(g_robot.cmd_queue, &rb_cmd, portMAX_DELAY) == pdPASS) {
		switch (rb_cmd.type)
		{
			case CMD_TYPE_UART1:
				robot_uart1_handle(&rb_cmd);
				break;
			
			case CMD_TYPE_MQTT:
				robot_mqtt_handle(&rb_cmd);
				break;
			
			default:
				break;
		}
	}
}

/* S 曲线路径生成：以 ROBOT_PID_PERIOD(10ms) 为步长，按正弦速度剖面生成笛卡尔路径点
 * 三段式（S ≥ 2·S_acc）：加速 + 匀速 + 减速
 * 两段式（S <  2·S_acc）：加速 + 减速（Vpeak 降低）
 * 输出写入 s_path_buf，返回生成点数（0 表示距离过小） */
static int robot_path_interpolation_scurve(struct position *target, int *size)
{
    float dx = target->x - g_robot.cur_pos.x;
    float dy = target->y - g_robot.cur_pos.y;
    float dz = target->z - g_robot.cur_pos.z;
    float S = sqrtf(dx*dx + dy*dy + dz*dz);

    if (S < 1e-3f) {
        s_path_buf[0] = *target;
        *size = 1;
        return 1;
    }

    float dir_x = dx / S, dir_y = dy / S, dir_z = dz / S;

    const float dt = ROBOT_PID_PERIOD / 1000.0f; /* 10ms */
    int n = 0;
    float t = 0.0f, s = 0.0f;

    if (S >= 2.0f * SCURVE_S_ACCEL) {
        /* 三段式 */
        float T_const = (S - 2.0f * SCURVE_S_ACCEL) / SCURVE_VMAX;
        float T_total = 2.0f * SCURVE_T_ACCEL + T_const;
        while (t <= T_total + dt * 0.5f && n < ROBOT_MAX_PATH_SIZE) {
            float v;
            if (t <= SCURVE_T_ACCEL) {
                v = (SCURVE_VMAX * 0.5f) * (sinf(SCURVE_OMEGA * t - 3.14159265f * 0.5f) + 1.0f);
            } else if (t <= SCURVE_T_ACCEL + T_const) {
                v = SCURVE_VMAX;
            } else {
                float t2 = t - SCURVE_T_ACCEL - T_const;
                v = (SCURVE_VMAX * 0.5f) * (sinf(3.14159265f * 0.5f - SCURVE_OMEGA * t2) + 1.0f);
            }
            s += v * dt;
            if (s > S) s = S;
            s_path_buf[n].x = g_robot.cur_pos.x + dir_x * s;
            s_path_buf[n].y = g_robot.cur_pos.y + dir_y * s;
            s_path_buf[n].z = g_robot.cur_pos.z + dir_z * s;
            n++;
            t += dt;
        }
    } else {
        /* 两段式：降低峰值速度 */
        float Vpeak = sqrtf(S * SCURVE_AMAX * 3.14159265f * 0.5f);
        float omega2 = 2.0f * SCURVE_AMAX / Vpeak;
        float T_half = 3.14159265f / omega2;
        float T_total = 2.0f * T_half;
        while (t <= T_total + dt * 0.5f && n < ROBOT_MAX_PATH_SIZE) {
            float v;
            if (t <= T_half) {
                v = (Vpeak * 0.5f) * (sinf(omega2 * t - 3.14159265f * 0.5f) + 1.0f);
            } else {
                float t2 = t - T_half;
                v = (Vpeak * 0.5f) * (sinf(3.14159265f * 0.5f - omega2 * t2) + 1.0f);
            }
            s += v * dt;
            if (s > S) s = S;
            s_path_buf[n].x = g_robot.cur_pos.x + dir_x * s;
            s_path_buf[n].y = g_robot.cur_pos.y + dir_y * s;
            s_path_buf[n].z = g_robot.cur_pos.z + dir_z * s;
            n++;
            t += dt;
        }
    }

    /* 确保末点精确 */
    if (n > 0) {
        s_path_buf[n - 1] = *target;
    } else {
        s_path_buf[0] = *target;
        n = 1;
    }

    *size = n;
    return n;
}

static int robot_update_current_angle(uint8_t joint_id)
{
    vTaskDelay(pdMS_TO_TICKS(20));
//    LOG("robot_update_current_angle START!!!\r\n");


    uint8_t addr = (uint8_t)(joint_id + 1u);

    if (!BSP_CAN_Lock(200)) {
        LOG("joint %u update current angle: CAN bus lock timeout.\r\n", joint_id);
        return 1;
    }

    BSP_CAN_DrainRx(); /* 排空锁前残留帧，避免0xFD广播帧消耗0x36应答窗口 */

    /* 发送读取电机当前位置的CAN命令 */
    Emm_V5_Read_Sys_Params(addr, S_CPOS);

    uint8_t rx[8] = {0};
    uint8_t dlc = 0;
    uint32_t ext_id = 0;
    if (!BSP_CAN_WaitReply(addr, 0x36u, rx, &dlc, 150u, &ext_id))
    {
        BSP_CAN_Unlock();
        LOG("joint %u update current angle timeout.\r\n", joint_id);
        return 1;
    }

    BSP_CAN_Unlock();

    uint8_t id = (uint8_t)(ext_id >> 8) - 1u;
    if ((dlc < 7u) || (rx[0] != 0x36u) || (rx[dlc - 1u] != 0x6Bu) || (id != joint_id))
    {
        LOG("CAN RX validation failed! Joint: %u, ID: %u, Data[0]: 0x%02x\r\n", joint_id, id, rx[0]);
        return 1;
    }

    return robot_update_current_angle_from_data(joint_id, addr, rx, dlc);
}

static int robot_joint_stop(uint8_t joint_id)
{
    uint8_t addr = (uint8_t)(joint_id + 1u);

    if (!BSP_CAN_Lock(200)) {
        LOG("joint %u stop: CAN bus lock timeout.\n", joint_id);
        return 1;
    }

    Emm_V5_Stop_Now(addr, false);

    uint8_t rx[8] = {0};
    uint8_t dlc = 0;
    if (!BSP_CAN_WaitReply(addr, 0xFEu, rx, &dlc, 50u, NULL))
    {
        BSP_CAN_Unlock();
        LOG("joint %u stop timeout.\n", joint_id);
        return 1;
    }

    BSP_CAN_Unlock();
	g_robot.joints[joint_id].velocity = 0;
	return 0;
}

/* 批量停止所有关节：并发发送 stop 指令，统一等待 0xFE 应答，避免顺序等待漏帧 */
static void robot_joint_stop_all(uint8_t joint_num)
{
    if (joint_num > ROBOT_MAX_JOINT_NUM) joint_num = (uint8_t)ROBOT_MAX_JOINT_NUM;
    if (joint_num == 0u) return;

    if (!BSP_CAN_Lock(200)) return;

    BSP_CAN_ClearStopFlags(joint_num);
    for (uint8_t j = 0u; j < joint_num; j++) {
        Emm_V5_Stop_Now((uint8_t)(j + 1u), false);
        g_robot.joints[j].velocity = 0;
    }

    BSP_CAN_WaitStopAll(joint_num, 100u);
    BSP_CAN_Unlock();
}

static void robot_joint_stop_from_isr(uint8_t joint_id)
{
	Emm_V5_Stop_Now((uint8_t)(joint_id + 1u), false);
	g_robot.joints[joint_id].velocity = 0;
}

static int time_func_circle(uint32_t time_ms, struct position *pos)
{
	float angle_vel = 2 * M_PI / 5; // 角速度，5s转一圈，配合600点路径缓冲
	float r = g_time_func_circle_radius_mm;
	const float entry_ms = 1000.0f;
	const float circle_x = 0.0f;
	const float circle_y = -70.0f;
	const float circle_z = 0.0f;
	
	if ((float)time_ms < entry_ms) {	// 前1S移动到画圆起点
		float k = (float)time_ms / entry_ms;
		pos->x = g_robot.cur_pos.x + (circle_x - g_robot.cur_pos.x) * k;
		pos->y = g_robot.cur_pos.y + (circle_y - g_robot.cur_pos.y) * k;
		pos->z = g_robot.cur_pos.z + (circle_z - g_robot.cur_pos.z) * k;
		return 0;
	}
	
	time_ms -= 1000;
	// 保持 Y=-50，在 X/Z 平面按当前半径画圆
	float t = (float)time_ms / 1000.0f;
	pos->x = circle_x + r * sinf(angle_vel * t);
	pos->y = circle_y;
	pos->z = circle_z + r * (1.0f - cosf(angle_vel * t));
	return 0;
}

