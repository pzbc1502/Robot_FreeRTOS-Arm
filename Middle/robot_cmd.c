#include "robot.h"
#include "bsp_uart.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include "robot_workflow.h"
#include "robot_capture.h"
#include "robot_cmd.h"
#include "Emm_V5.h"
#include "gripper.h"
#include "bsp_laser.h"
#include "robot_target.h"

/* --- gripper debug commands (UART1) --- */
static int robot_soft_reset_handle(float *param);
static int robot_abs_rotate_handle(float *param);
static int robot_auto_handle(float *param);
static int robot_rel_rotate_handle(float *param);
static int robot_read_all_handle(float *param);
static int robot_gripper_stop_handle(float *param);
static int robot_gripper_open_handle(float *param);
static int robot_gripper_cur_handle(float *param);
static int robot_gripper_grasp_handle(float *param);
static int robot_laser_on_handle(float *param);
static int robot_laser_off_handle(float *param);
static int robot_target_enable_handle(float *param);
static int robot_target_disable_handle(float *param);
static int robot_time_func_handle(float *param);
static int robot_view_arc_handle(float *param);
static int robot_motion_abort_handle(float *param);


static bool debug_command_allowed(void)
{
    if ((robot_workflow_debug_epoch() == 0u) || robot_is_auto_busy() ||
        robot_capture_is_active() || robot_is_visual_servo_active()) {
        LOG("debug motion rejected: workflow or motion owns the robot\r\n");
        return false;
    }
    return true;
}

static bool valid_joint_id(float value)
{
    return isfinite(value) && value >= 0.0f && value < ROBOT_MAX_JOINT_NUM && floorf(value) == value;
}

void robot_mqtt_handle(struct robot_cmd *cmd)
{
	float param[6] = {0};
	int strlen = 0;
	int type = 0;
	
	LOG("robot mqtt cmd: %s\n", cmd->cmd);

	// 协议格式示例: [MCU][TYPE][ARG0-5]
	int result = sscanf(cmd->cmd, "+MQTTSUBRECV:0,\"arm/change\",%d,[MCU][%d][%f %f %f %f %f %f]", &strlen, &type,
			&param[0], &param[1], &param[2],
			&param[3], &param[4], &param[5]);
	
	if ((result != 8) || !debug_command_allowed()) { // 解析失败
		return;
	}
	
    for (unsigned i = 0u; i < 6u; i++) {
        if (!isfinite(param[i])) return;
    }
    if ((type == ROBOT_JOINT_ABS_ROTATE) && !valid_joint_id(param[0])) return;
	switch (type)
	{
		case ROBOT_JOINT_ABS_ROTATE:
			robot_abs_rotate_handle(param);
			break;
		
		case ROBOT_AUTO_EVENT:
			robot_auto_handle(param);
			break;
		
		case ROBOT_JOINTS_SYNC_EVENT:
			robot_auto_handle(param);
			break;
		
		default:
			break;
	}
}

static int robot_remote_enable_handle(float *param)
{
	robot_soft_reset_handle(param);	/* 复位 */
	ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_RMODE_ENABLE);
	return robot_send_remote_event();
}

static int robot_remote_disable_handle(float *param)
{
	(void)param;
	ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_RMODE_ENABLE);
	robot_soft_reset_handle(param);	/* 复位 */
	return pdPASS;	
}

static int robot_rel_rotate_handle(float *param)
{
	uint8_t joint_id = (uint8_t)param[0];
	return robot_send_rel_rotate_event(joint_id, param[1]);
}

static int robot_abs_rotate_handle(float *param)
{
	uint8_t joint_id = (uint8_t)param[0];
	return robot_send_abs_rotate_event(joint_id, param[1]);
}

static int robot_auto_handle(float *param)
{
	return robot_send_auto_event((struct position *)param);
}

static int robot_time_func_handle(float *param)
{
	float time_ms = (param[0] > 0.0f) ? param[0] : 6000.0f;
	float radius_mm = (param[1] > 0.0f) ? param[1] : 15.0f;
	LOG("time_func %.0f ms radius %.1f mm\r\n", time_ms, radius_mm);
	return robot_send_time_func_event(time_ms, radius_mm);
}

static int robot_view_arc_handle(float *param)
{
    float duration_ms = (param[0] > 0.0f) ? param[0] : 8000.0f;
    LOG("view_arc %.0f ms\r\n", duration_ms);
    return robot_send_view_arc_event(duration_ms);
}

static int robot_motion_abort_handle(float *param)
{
    (void)param;
    robot_motion_abort();
    LOG("motion_abort latched; soft_reset required\r\n");
    return pdPASS;
}

static int robot_hard_reset_handle(float *param)
{
	(void)param;
	return robot_send_reset_event(true);	
}

static int robot_soft_reset_handle(float *param)
{
	(void)param;
	return robot_send_reset_event(false);	
}

static int robot_read_all_handle(float *param)
{
    (void)param;
    return robot_send_read_all_event();
}

static int robot_remote_event_handle(float *param)
{
	if (!ROBOT_STATUS_IS(g_robot.status, ROBOT_STATUS_RMODE_ENABLE)) {
		return pdPASS;
	}

	float vx = -param[0] * ROBOT_REMOTE_MAX_VELOCITY;
	float vy = param[1] * ROBOT_REMOTE_MAX_VELOCITY;
	float vz = (param[4] - param[5]) / 2 * ROBOT_REMOTE_MAX_VELOCITY;
	float rx = -param[3] * ROBOT_REMOTE_MAX_RPM;
	float ry = param[2] * ROBOT_REMOTE_MAX_RPM;
	
	taskENTER_CRITICAL();
	g_remote_control.vx = vx;
	g_remote_control.vy = vy;
	g_remote_control.vz = vz;
	g_remote_control.rx = rx;
	g_remote_control.ry = ry;
	taskEXIT_CRITICAL();

	return pdPASS;
}

static int robot_zero_handle(float *param)
{
    (void)param;
    LOG("zero rejected: encoder recalibration requires a dedicated procedure\r\n");
    return pdFAIL;
}

/* ===================== 夹爪联调命令 =====================
 * 串口助手输入示例：
 * - gripper_stop
 * - gripper_open
 * - gripper_cur
 * - gripper_grasp <fruit_type>
 *   例：gripper_grasp 0   (0=草莓,1=番茄,2=葡萄)
 */

static int robot_gripper_stop_handle(float *param)
{
	(void)param;
	gripper_result_t r = gripper_stop(6);
	LOG("gripper_stop r=%d\r\n", (int)r);
	return (r == GRIPPER_OK) ? pdPASS : pdFAIL;
}

static int robot_gripper_open_handle(float *param)
{
	(void)param;
	gripper_result_t r = gripper_open();
	LOG("gripper_open r=%d\r\n", (int)r);
	return (r == GRIPPER_OK) ? pdPASS : pdFAIL;
}

static int robot_gripper_cur_handle(float *param)
{
	(void)param;
	uint16_t ma = 0u;
	gripper_result_t r = gripper_read_phase_current_ma(0, &ma);
	LOG("gripper_current=%u mA r=%d\r\n", (unsigned)ma, (int)r);
	return (r == GRIPPER_OK) ? pdPASS : pdFAIL;
}

static int robot_gripper_grasp_handle(float *param)
{
	gripper_fruit_t type = (gripper_fruit_t)((uint8_t)param[0]);
	gripper_grasp_cfg_t cfg = {0};

	if (!gripper_get_preset(type, &cfg)) {
		LOG("gripper_grasp invalid type=%u\r\n", (unsigned)type);
		return pdFAIL;
	}

	uint16_t touch_ma = 0u;
	gripper_result_t r = gripper_grasp_force(&cfg, &touch_ma);
	LOG("gripper_grasp type=%u limit=%u touch=%u r=%d\r\n",
		(unsigned)type, (unsigned)cfg.current_limit_ma, (unsigned)touch_ma, (int)r);
	return (r == GRIPPER_OK) ? pdPASS : pdFAIL;
}

static int robot_laser_on_handle(float *param)
{
	(void)param;
	BSP_Laser_Off();
	LOG("laser_on rejected: formal workflow safety gate required.\r\n");
	return pdFAIL;
}

static int robot_laser_off_handle(float *param)
{
	(void)param;
	BSP_Laser_Off();
	LOG("laser_off\r\n");
	return pdPASS;
}

static int robot_target_enable_handle(float *param)
{
	(void)param;
	BSP_Laser_Off();
	LOG("target_enable rejected: use formal WORKFLOW_CTRL and TARGET_CTRL.\r\n");
	return pdFAIL;
}

static int robot_target_disable_handle(float *param)
{
	(void)param;
	BSP_Laser_Off();
	robot_target_disable_request();
	LOG("target_disable: target substate held, laser forced off.\r\n");
	return pdPASS;
}

static struct robot_cmd_info robot_uart1_cmd_table[] = {
	/* 远程控制指令 */
	{"remote_event",   robot_remote_event_handle, 6u, 6u},
	{"remote_enable",  robot_remote_enable_handle, 0u, 0u},
	{"remote_disable", robot_remote_disable_handle, 0u, 0u},

	/* 机械臂控制指令 */
	{"abs_rotate",  robot_abs_rotate_handle, 2u, 2u},
	{"rel_rotate",  robot_rel_rotate_handle, 2u, 2u},
	{"auto",        robot_auto_handle, 3u, 3u},
    {"view_arc",   robot_view_arc_handle, 0u, 1u},
	{"circle",      robot_time_func_handle, 0u, 2u},
	{"time_func",   robot_time_func_handle, 0u, 2u},
	{"hard_reset",  robot_hard_reset_handle, 0u, 0u},
	{"soft_reset",  robot_soft_reset_handle, 0u, 0u},
    {"motion_abort", robot_motion_abort_handle, 0u, 0u},
    {"read_all",    robot_read_all_handle, 0u, 0u},
	{"zero",        robot_zero_handle, 0u, 0u},

	/* 夹爪控制指令 */
	{"gripper_stop",  robot_gripper_stop_handle, 0u, 0u},
	{"gripper_open",  robot_gripper_open_handle, 0u, 0u},
	{"gripper_cur",   robot_gripper_cur_handle, 0u, 0u},
	{"gripper_grasp", robot_gripper_grasp_handle, 1u, 1u},

	/* 激光控制指令 */
	{"target_enable",  robot_target_enable_handle, 0u, 0u},
	{"target_disable", robot_target_disable_handle, 0u, 0u},
	{"laser_on",       robot_laser_on_handle, 0u, 0u},
	{"laser_off",      robot_laser_off_handle, 0u, 0u},
	{NULL, NULL, 0u, 0u},
};

void robot_uart1_handle(struct robot_cmd *rb_cmd)
{
	float param[6] = {0};
	char *cmd = rb_cmd->cmd;
	int ret;

	// Trim leading whitespace
	while (*cmd == ' ' || *cmd == '\t') {
		cmd++;
	}

	for (int i = 0; robot_uart1_cmd_table[i].event_type != NULL; i++) {
		const char *event = robot_uart1_cmd_table[i].event_type;
		size_t len = strlen(event);
		// Check if the command starts with the event string
		if (strncmp(cmd, event, len) == 0) {
			// Check if it's a full match (followed by space, null terminator, or newline)
			char next_char = cmd[len];
			if (next_char == ' ' || next_char == '\0' || next_char == '\r' || next_char == '\n') {
				
                char *cursor = cmd + len;
                unsigned count = 0u;
                while (1) {
                    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' || *cursor == '\n') cursor++;
                    if (*cursor == '\0') break;
                    char *end = NULL;
                    errno = 0;
                    if (count >= 6u) return;
                    float value = strtof(cursor, &end);
                    if ((end == cursor) || (errno == ERANGE) || !isfinite(value) ||
                        (*end != '\0' && *end != ' ' && *end != '\t' && *end != '\r' && *end != '\n')) {
                        LOG("invalid debug command parameter\r\n");
                        return;
                    }
                    param[count++] = value;
                    cursor = end;
                }
                const struct robot_cmd_info *info = &robot_uart1_cmd_table[i];
                if ((count < info->min_args) || (count > info->max_args)) {
                    LOG("wrong debug command argument count\r\n");
                    return;
                }
                if ((info->cmd_func == robot_abs_rotate_handle || info->cmd_func == robot_rel_rotate_handle) &&
                    !valid_joint_id(param[0])) return;
                if (info->cmd_func == robot_gripper_grasp_handle &&
                    (param[0] < 0.0f || param[0] >= (float)GRIPPER_FRUIT_MAX || floorf(param[0]) != param[0])) return;
                bool safe_command = info->cmd_func == robot_motion_abort_handle ||
                    info->cmd_func == robot_laser_off_handle || info->cmd_func == robot_target_disable_handle ||
                    info->cmd_func == robot_read_all_handle || info->cmd_func == robot_gripper_cur_handle ||
                    info->cmd_func == robot_gripper_stop_handle || info->cmd_func == robot_laser_on_handle ||
                    info->cmd_func == robot_target_enable_handle;
                if (!safe_command && !debug_command_allowed()) return;

				// Execute command
				LOG("Found handler for '%s', executing...\n", event);
				ret = robot_uart1_cmd_table[i].cmd_func(param);
				if (ret != pdPASS) {
					LOG("[ERROR] event_type:%s failed\n", event);
				}
				return; // Command found and executed, exit function
			}
		}
	}

	LOG("uart cmd parse error: '%s'\n", rb_cmd->cmd);
}

