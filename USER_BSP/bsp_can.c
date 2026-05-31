#include "bsp_can.h"
#include <string.h>
#include <stdio.h>
#include "robot.h"
#include "bsp_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* CAN 总线互斥锁（串行化请求-应答） */
static SemaphoreHandle_t s_can_bus_mutex = NULL;

/* 全局 CAN 上下文结构体，用于保存接收/发送缓冲区和状态 */
CAN_Context_t g_can_context = {0};
volatile bool g_can_tx_complete = false; // CAN 发送完成标志

/* ============================================================ */
/*                1. 接收规则配置 (使用宏定义)                  */
/* ============================================================ */
const canfd_afl_entry_t p_canfd0_afl[] = 
{
    {
        .id = 
        {
            /* 配置使用通配符模式接收所有扩展帧 */
            .id         = 0,
            .id_mode    = CAN_ID_MODE_EXTENDED,  // 使用扩展ID模式
            .frame_type = CAN_FRAME_TYPE_DATA    // 仅接收数据帧
        },
        .mask = 
        {
            .mask_id    = 0,  // 0表示不进行ID过滤
        },
        .destination = 
        {
            /* 
             * 硬件路由配置:
             * 接收数据将存入 RX FIFO 0
             * 配置路径: FSP -> Stacks -> r_canfd -> Common -> Reception
             */
             .fifo_select_flags  = CANFD_RX_FIFO_0 
        }
    },
};


/* ============================================================ */
/*                        初始化函数                            */
/* ============================================================ */
void BSP_CAN_Init(void)
{
    fsp_err_t err;

    /* 打开并初始化 CAN 模块 */
    err = g_canfd0.p_api->open(g_canfd0.p_ctrl, g_canfd0.p_cfg);

    /* 创建 CAN 接收信号量 */
    g_can_context.can_rx_sem = xSemaphoreCreateBinary();

    /* 创建 CAN 总线互斥锁（用于串行化请求-应答） */
    if (s_can_bus_mutex == NULL) {
        s_can_bus_mutex = xSemaphoreCreateMutex();
    }

    if ((FSP_SUCCESS != err) || (g_can_context.can_rx_sem == NULL) || (s_can_bus_mutex == NULL))
    {
        __BKPT(0);  // 初始化失败时触发断点
    }
}

/* 简易状态打印，便于错误定位 */
static void BSP_CAN_PrintInfo(const char *tag)
{
    can_info_t info = {0};
    if (R_CANFD_InfoGet(&g_canfd0_ctrl, &info) == FSP_SUCCESS)
    {
        LOG("[CAN][%s] status=0x%08lX tx_err=%u rx_err=%u err_code=0x%08lX\r\n",
               tag ? tag : "info",
               (unsigned long) info.status,
               (unsigned int) info.error_count_transmit,
               (unsigned int) info.error_count_receive,
               (unsigned long) info.error_code);
    }
}

/* ============================================================ */
/*                      底层发送函数 (最终版)                   */
/* ============================================================ */
fsp_err_t BSP_CAN_Send(uint32_t id, can_id_mode_t id_mode, uint8_t *p_data, uint8_t len)
{
    fsp_err_t err;
    can_frame_t tx_frame;

    tx_frame.id                = id;
    tx_frame.id_mode           = id_mode; // 关键：确保ID模式被正确传入和设置
    tx_frame.type              = CAN_FRAME_TYPE_DATA;
    tx_frame.data_length_code  = len;
    tx_frame.options           = 0; // 标准CAN 2.0帧，不使用BRS或FDF
    memcpy(tx_frame.data, p_data, len);

    g_can_tx_complete = false; // 发送前清除标志
    err = R_CANFD_Write(&g_canfd0_ctrl, 0, &tx_frame);
    if (err != FSP_SUCCESS) {
        LOG("[CAN][TX] write failed id=0x%08lX dlc=%u err=%d\r\n", (unsigned long) id, (unsigned int) len, err);
        BSP_CAN_PrintInfo("write_fail");
        return err; // 如果写入buffer失败，直接返回错误
    }

    /* 等待发送完成中断标志 */
    uint32_t timeout = 0xFFFFF;
    while (!g_can_tx_complete && (timeout-- > 0))
    {
        __NOP();
    }

    if (g_can_tx_complete)
    {
        return FSP_SUCCESS;
    }

    LOG("[CAN][TX] timeout id=0x%08lX dlc=%u\r\n", (unsigned long) id, (unsigned int) len);
    BSP_CAN_PrintInfo("timeout");
    return FSP_ERR_TIMEOUT; // 如果是超时，返回超时错误
}

/* ============================================================ */
/*                  上层应用接口 (最终版-支持分包)              */
/* ============================================================ */
void can_SendCmd(uint8_t *cmd, uint8_t len)
{
    if (len < 2) return; // 命令至少需要地址+功能码

    uint32_t motor_id = cmd[0];
    uint8_t func = cmd[1];
    uint8_t data_len = (uint8_t)(len - 2); // 除去地址和功能码后的数据长度

    uint8_t offset = 0;
    uint8_t packNum = 0;

    do
    {
        uint8_t remaining = (data_len > offset) ? (uint8_t)(data_len - offset) : 0;
        uint8_t dlc = (remaining < 7) ? (uint8_t)(remaining + 1) : 8; // 1字节功能码 + 最多7字节数据

        uint8_t txbuf[8] = {0};
        txbuf[0] = func;
        for (uint8_t i = 0; i < (uint8_t)(dlc - 1); i++)
        {
            txbuf[i + 1] = cmd[2 + offset + i];
        }

        uint32_t ext_id = ((motor_id << 8) | packNum);
        fsp_err_t err = BSP_CAN_Send(ext_id, CAN_ID_MODE_EXTENDED, txbuf, dlc);
        if (err != FSP_SUCCESS)
        {
            LOG("CAN Send chunk for Motor ID %u failed with err: %d\r\n", (unsigned int) motor_id, err);
            return;
        }

        offset = (uint8_t)(offset + (dlc - 1));
        packNum++;
    } while (offset < data_len);
}

/* ============================================================ */
/*                CAN 应答等待与总线互斥(请求-应答)               */
/* ============================================================ */
bool BSP_CAN_Lock(uint32_t timeout_ms)
{
    if (s_can_bus_mutex == NULL)
    {
        s_can_bus_mutex = xSemaphoreCreateMutex();
        if (s_can_bus_mutex == NULL) {
            return false;
        }
    }

    /* 调度器未启动时不做阻塞锁（按“单线程”方式运行） */
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return true;
    }

    return (pdTRUE == xSemaphoreTake(s_can_bus_mutex, pdMS_TO_TICKS(timeout_ms)));
}

void BSP_CAN_Unlock(void)
{
    if (s_can_bus_mutex != NULL) {
        (void) xSemaphoreGive(s_can_bus_mutex);
    }
}

bool BSP_CAN_WaitReply(uint8_t addr,
                      uint8_t expected_func,
                      uint8_t *out_buf,
                      uint8_t *out_dlc,
                      uint32_t timeout_ms,
                      uint32_t *out_ext_id)
{
    if ((out_buf == NULL) || (out_dlc == NULL)) {
        return false;
    }
    if (g_can_context.can_rx_sem == NULL) {
        return false;
    }

    TickType_t start = xTaskGetTickCount();
    TickType_t deadline = start + pdMS_TO_TICKS(timeout_ms);

    while (1)
    {
        TickType_t now = xTaskGetTickCount();
        if (now >= deadline) {
            return false;
        }

        TickType_t wait_ticks = deadline - now;
        if (pdTRUE != xSemaphoreTake(g_can_context.can_rx_sem, wait_ticks)) {
            return false;
        }

        uint32_t ext_id;
        uint8_t dlc;
        uint8_t data[8] = {0};

        __disable_irq();
        ext_id = g_can_context.CAN_RxMsg.ExtId;
        dlc = (uint8_t) g_can_context.CAN_RxMsg.DLC;
        if (dlc > 8u) {
            dlc = 8u;
        }
        for (uint8_t i = 0; i < dlc; i++) {
            data[i] = g_can_context.rxData[i];
        }
        __enable_irq();

        uint8_t rx_addr = (uint8_t) (ext_id >> 8);
        if (rx_addr != addr) {
            continue;
        }
        if (dlc < 2u) {
            continue;
        }
        if (data[0] != expected_func) {
            continue;
        }

        /* 多数回包以 0x6B 结尾（校验字节），做弱校验 */
        if (data[dlc - 1u] != 0x6Bu) {
            continue;
        }

        for (uint8_t i = 0; i < dlc; i++) {
            out_buf[i] = data[i];
        }
        *out_dlc = dlc;
        if (out_ext_id != NULL) {
            *out_ext_id = ext_id;
        }
        return true;
    }
}

/* ============================================================ */
/*                      中断回调函数                            */
/* ============================================================ */
void canfd0_callback(can_callback_args_t *p_args)
{
    uint8_t jointId = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (p_args->event)
    {
        case CAN_EVENT_RX_COMPLETE:
        {
            /* 
             * 接收完成中断回调
             * p_args->frame 包含接收到的完整帧数据
             */

            // 保存帧ID和数据长度
            g_can_context.CAN_RxMsg.ExtId = p_args->frame.id;
            g_can_context.CAN_RxMsg.DLC   = p_args->frame.data_length_code;

            // 限制最大拷贝8字节 (标准CAN帧)
            uint8_t copy_len = (uint8_t)((g_can_context.CAN_RxMsg.DLC > 8) ? 8 : g_can_context.CAN_RxMsg.DLC);
            memcpy(g_can_context.rxData, p_args->frame.data, copy_len);

            /* --- 业务逻辑处理 --- */
            // 从ID解析关节ID (ID高8位)
            jointId = (uint8_t)(g_can_context.CAN_RxMsg.ExtId >> 8) - 1;

            // 检测关节就绪特征码: 0xFD 0x9F 0x6B
            if ((g_can_context.CAN_RxMsg.DLC == 3) &&
                (g_can_context.rxData[0] == 0xFD) &&
                (g_can_context.rxData[1] == 0x9F) &&
                (g_can_context.rxData[2] == 0x6B))
            {
                // 有效性校验 (防止数组越界)
                if (jointId < ROBOT_MAX_JOINT_NUM)
                {
                    // 设置关节状态为就绪
                    ROBOT_STATUS_SET(g_robot.joints[jointId].status, ROBOT_STATUS_READY);
                }
            }

            // 释放信号量，通知等待的任务
            if (g_can_context.can_rx_sem != NULL)
            {
                xSemaphoreGiveFromISR(g_can_context.can_rx_sem, &xHigherPriorityTaskWoken);
            }

            /* 如果释放信号量导致一个更高优先级的任务被唤醒，则进行一次任务切换 */
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;
        }

        case CAN_EVENT_TX_COMPLETE:
        {
            g_can_tx_complete = true; // 设置发送完成标志
            break;
        }

        case CAN_EVENT_ERR_BUS_OFF:
        case CAN_EVENT_ERR_PASSIVE:
        case CAN_EVENT_ERR_WARNING:
        case CAN_EVENT_BUS_RECOVERY:
        case CAN_EVENT_ERR_BUS_LOCK:
        {
            LOG("[CAN][ISR] event=%d id=0x%08lX dlc=%u\r\n", p_args->event, (unsigned long)p_args->frame.id, (unsigned int)p_args->frame.data_length_code);
            BSP_CAN_PrintInfo("err_isr");
            break;
        }
        
        default:
            // 其他事件不做处理
            break;
    }
}


