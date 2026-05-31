#include "K230_cmd.h"
#include "hal_data.h"
#include "bsp_uart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

// --- FSP生成的UART实例 ---
extern sci_uart_instance_ctrl_t robot_k230_ctrl;  // K230通信UART控制块
extern const uart_cfg_t robot_k230_cfg;            // K230通信UART配置

/* 根据项目配置，选择DMA或DTC传输实例 */
#if defined(g_dma_k230_rx)
extern const transfer_instance_t g_dma_k230_rx;   // DMA传输实例
#define K230_TRANSFER_INSTANCE g_dma_k230_rx       // 定义传输实例别名
#else
extern const transfer_instance_t g_transfer_k230_rx; // DTC传输实例
#define K230_TRANSFER_INSTANCE g_transfer_k230_rx    // 定义传输实例别名
#endif


// --- 静态变量 ---
static protocol_parser_t g_parser = { .state = PARSER_STATE_WAIT_FOR_SOF }; // 协议解析器状态机

static k230_point_t g_latest_point;        // 存储最新解析的坐标点
static volatile bool g_new_point_flag = false; // 坐标点更新标志

static k230_strawberry_count_t g_latest_count;   // 成熟/未成熟数量
static volatile bool g_new_count_flag = false;    // 数量更新标志

static k230_frame_t g_latest_frame;        // 存储最新解析的完整帧
static volatile bool g_new_frame_flag = false; // 帧更新标志

static uint8_t g_latest_harvestable = 0u;       // 0x04 最新值
static volatile bool g_new_harvestable_flag = false;


static uint8_t g_k230_rx_buffer[1024];     // UART接收环形缓冲区
static uint32_t g_last_read_pos = 0;       // 上次读取位置指针

static SemaphoreHandle_t g_k230_tx_sem = NULL;
static uint8_t g_k230_tx_frame[4];

static void process_byte(uint8_t byte);   // 前向声明：字节处理函数

/**
 * @brief 重置协议解析器状态
 * 将解析器恢复到初始等待帧起始符(SOF)的状态
 */
static void reset_parser(void)
{
    g_parser.state = PARSER_STATE_WAIT_FOR_SOF; // 重置状态
    g_parser.sof = 0;                           // 清除SOF标记
    g_parser.eof = 0;                           // 清除EOF标记
}

/**
 * @brief 处理有效帧数据
 * 将解析完成的帧数据缓存到全局变量，并设置更新标志
 * 特别处理坐标数据帧，转换为结构化坐标点
 */
static void handle_valid_frame(void)
{
    // 计算有效负载长度（总长度-1字节功能码）
    uint8_t payload_len = (uint8_t)(g_parser.len - 1u);
    if (payload_len > MAX_PAYLOAD_SIZE) { // 防止缓冲区溢出
        payload_len = MAX_PAYLOAD_SIZE;
    }

    /* 将解析的帧数据缓存到全局变量（原子操作） */
    __disable_irq(); // 禁用中断确保数据一致性
    g_latest_frame.sof = g_parser.sof;          // 帧起始符
    g_latest_frame.func = g_parser.func;        // 功能码
    g_latest_frame.payload_len = payload_len;   // 有效负载长度
    if (payload_len > 0) {
        memcpy(g_latest_frame.payload, g_parser.payload_buffer, payload_len); // 复制有效负载
    }
    g_new_frame_flag = true; // 设置帧更新标志
    __enable_irq(); // 重新启用中断

    /* 特殊处理：坐标数据帧 */
    if (g_parser.func == K230_FUNC_COORDINATE_DATA)
    {
        // 验证数据长度是否匹配坐标结构
        if ((g_parser.len - 1u) == sizeof(k230_point_t))
        {
#if (K230_COORD_ENDIAN == K230_COORD_ENDIAN_BIG)
            // 大端模式：高位字节在前
            g_latest_point.x = (int16_t)((g_parser.payload_buffer[0] << 8) | g_parser.payload_buffer[1]);
            g_latest_point.y = (int16_t)((g_parser.payload_buffer[2] << 8) | g_parser.payload_buffer[3]);
#else
            // 小端模式：低位字节在前
            g_latest_point.x = (int16_t)((g_parser.payload_buffer[1] << 8) | g_parser.payload_buffer[0]);
            g_latest_point.y = (int16_t)((g_parser.payload_buffer[3] << 8) | g_parser.payload_buffer[2]);
#endif
            g_new_point_flag = true; // 设置坐标更新标志
        }
    }
    else if (g_parser.func == K230_FUNC_HARVESTABLE)
    {
        if (payload_len >= 1u)
        {
            g_latest_harvestable = (g_parser.payload_buffer[0] == 0x01u) ? 1u : 0u;
            g_new_harvestable_flag = true;
        }
    }
    else if (g_parser.func == K230_FUNC_STRAWBERRY_COUNT)
    {
        /* 长度约定：LEN=3 (func+成熟+未成熟)，payload_len=2 */
        if (payload_len >= 2u)
        {
            g_latest_count.ripe   = g_parser.payload_buffer[0];
            g_latest_count.unripe = g_parser.payload_buffer[1];
            g_new_count_flag = true;
        }
    }
}


/**
 * @brief 处理单个字节的协议解析
 * 实现状态机驱动的协议解析
 * @param byte 待处理的字节
 */
static void process_byte(uint8_t byte)
{
    switch (g_parser.state)
    {
        case PARSER_STATE_WAIT_FOR_SOF:
            // 等待帧起始符(SOF)
            if (byte == K230_SOF_COORD) // 坐标数据帧SOF
            {
                g_parser.sof = byte;
                g_parser.eof = K230_EOF_COORD; // 设置对应EOF
                g_parser.state = PARSER_STATE_READ_LEN; // 转入长度解析状态
                g_parser.calculated_checksum = 0; // 重置校验和
                g_parser.payload_index = 0;        // 重置负载索引
            }
            else if (byte == K230_SOF_JUDGE) // 裁判系统帧SOF
            {
                g_parser.sof = byte;
                g_parser.eof = K230_EOF_JUDGE; // 设置对应EOF
                g_parser.state = PARSER_STATE_READ_LEN;
                g_parser.calculated_checksum = 0;
                g_parser.payload_index = 0;
            }
            else if (byte == K230_SOF_COUNT) // 草莓数量帧 SOF
            {
                g_parser.sof = byte;
                g_parser.eof = K230_EOF_COUNT;
                g_parser.state = PARSER_STATE_READ_LEN;
                g_parser.calculated_checksum = 0;
                g_parser.payload_index = 0;
            }
            break;


        case PARSER_STATE_READ_LEN:
            // 读取帧长度字节
            g_parser.len = byte;
            g_parser.calculated_checksum = (uint8_t)(g_parser.calculated_checksum + byte);

            /* 验证长度有效性：至少1字节(功能码)，最大不超过负载限制+1 */
            if (g_parser.len == 0u || g_parser.len > (MAX_PAYLOAD_SIZE + 1u))
            {
                reset_parser(); // 无效长度，重置解析器
            }
            else
            {
                g_parser.state = PARSER_STATE_READ_FUNC; // 转入功能码解析
            }
            break;

        case PARSER_STATE_READ_FUNC:
            // 读取功能码
            g_parser.func = byte;
            g_parser.calculated_checksum = (uint8_t)(g_parser.calculated_checksum + byte);
            // 根据长度决定下一步：有负载则读负载，否则读校验和
            g_parser.state = (g_parser.len > 1u) ? PARSER_STATE_READ_PAYLOAD : PARSER_STATE_READ_CHECKSUM;
            break;

        case PARSER_STATE_READ_PAYLOAD:
            // 读取有效负载
            g_parser.payload_buffer[g_parser.payload_index++] = byte;
            g_parser.calculated_checksum = (uint8_t)(g_parser.calculated_checksum + byte);

            // 检查是否读取完所有负载
            if (g_parser.payload_index >= (uint8_t)(g_parser.len - 1u))
            {
                g_parser.state = PARSER_STATE_READ_CHECKSUM; // 转入校验和解析
            }
            break;

        case PARSER_STATE_READ_CHECKSUM:
            // 验证校验和
            if (byte == g_parser.calculated_checksum)
            {
                g_parser.state = PARSER_STATE_READ_EOF; // 校验成功，等待EOF
            }
            else
            {
                reset_parser(); // 校验失败，重置解析器
            }
            break;

        case PARSER_STATE_READ_EOF:
            // 验证帧结束符(EOF)
            if (byte == g_parser.eof)
            {
                handle_valid_frame(); // 完整帧解析成功
            }
            reset_parser(); // 无论成功与否，重置解析器
            break;

        default:
            reset_parser(); // 未知状态，重置
            break;
    }
}

/**
 * @brief 初始化K230通信协议
 * 配置UART外设并启动DMA/DTC接收
 */
void k230_protocol_init(void)
{
    fsp_err_t err;

    if (g_k230_tx_sem == NULL) {
        g_k230_tx_sem = xSemaphoreCreateBinary();
    }

    // 打开UART外设
    err = R_SCI_UART_Open(&robot_k230_ctrl, &robot_k230_cfg);
    if (FSP_SUCCESS != err) {
        __BKPT(0); // 初始化失败时触发断点
    }

    /* 启动DMA/DTC接收，使用环形缓冲区 */
    err = R_SCI_UART_Read(&robot_k230_ctrl, g_k230_rx_buffer, sizeof(g_k230_rx_buffer));
    if (FSP_SUCCESS != err)
    {
        LOG("K230: RX start failed\n"); // 记录错误日志
    }
}

/**
 * @brief 处理K230协议数据
 * 从DMA/DTC环形缓冲区读取新数据并解析
 * 应在主循环中定期调用
 */
void k230_protocol_process(void)
{
    uint32_t current_write_pos = 0;
    uint32_t rx_buf_size = (uint32_t) sizeof(g_k230_rx_buffer);

    /* 关键修复：不要用 transfer_info_t::p_dest 计算“当前写入位置”。
     * 对于 DMAC/DTC，这个字段通常是“配置值”，不会随着硬件写入实时变化；
     * 且在传输完成时可能变成 buffer+len(=1024)，会导致下面的 while(i!=pos) 永不结束。
     */
    transfer_properties_t props = {0};
    fsp_err_t err = K230_TRANSFER_INSTANCE.p_api->infoGet(K230_TRANSFER_INSTANCE.p_ctrl, &props);
    if (FSP_SUCCESS != err)
    {
        return;
    }

    /* 用“剩余字节数”推算已写入位置：pos = (buf_size - remaining) % buf_size */
    uint32_t remaining = props.transfer_length_remaining;
    if (remaining > rx_buf_size) {
        remaining = rx_buf_size;
    }
    current_write_pos = (rx_buf_size - remaining) % rx_buf_size;

    /* 如果传输已完成（remaining==0），说明缓冲区已经写满一次。
     * 先把剩余未读的数据(从last_read到buf末尾)解析掉，再重新启动一次 Read。
     */
    if (remaining == 0u)
    {
        uint32_t i = g_last_read_pos;
        while (i < rx_buf_size)
        {
            process_byte(g_k230_rx_buffer[i]);
            i++;
        }

        g_last_read_pos = 0u;
        (void) R_SCI_UART_Read(&robot_k230_ctrl, g_k230_rx_buffer, rx_buf_size);
        return;
    }

    /* 正常增量解析 */
    if (current_write_pos != g_last_read_pos)
    {
        uint32_t i = g_last_read_pos;
        while (i != current_write_pos)
        {
            process_byte(g_k230_rx_buffer[i]);
            i = (i + 1u) % rx_buf_size;
        }
        g_last_read_pos = current_write_pos;
    }
}


/**
 * @brief 获取最新坐标点
 * @param point 指向坐标结构的指针
 * @return true 有新坐标，false 无新数据
 * 线程安全：通过关中断保护共享数据
 */
bool k230_get_latest_coords(k230_point_t *point)
{
    if (g_new_point_flag)
    {
        __disable_irq(); // 禁用中断
        *point = g_latest_point; // 复制坐标数据
        g_new_point_flag = false; // 清除更新标志
        __enable_irq(); // 重新启用中断
        return true;
    }
    return false;
}

/**
 * @brief 获取最新完整帧
 * @param frame 指向帧结构的指针
 * @return true 有新帧，false 无新数据
 * 线程安全：通过关中断保护共享数据
 */
bool k230_get_latest_frame(k230_frame_t *frame)
{
    if (g_new_frame_flag)
    {
        __disable_irq(); // 禁用中断
        *frame = g_latest_frame; // 复制帧数据
        g_new_frame_flag = false; // 清除更新标志
        __enable_irq(); // 重新启用中断
        return true;
    }
    return false;
}

bool k230_get_latest_harvestable(uint8_t *value)
{
    if (value == NULL) {
        return false;
    }

    if (g_new_harvestable_flag)
    {
        __disable_irq();
        *value = g_latest_harvestable;
        g_new_harvestable_flag = false;
        __enable_irq();
        return true;
    }

    return false;
}

/**
 * @brief 获取最新数量
 * @param point 指向数量结构的指针
 * @return true 有新数量，false 无新数量
 * 线程安全：通过关中断保护共享数据
 */
bool k230_get_latest_strawberry_count(k230_strawberry_count_t *count)
{
    if (count == NULL) {
        return false;
    }
    if (g_new_count_flag)
    {
        __disable_irq();
        *count = g_latest_count;
        g_new_count_flag = false;
        __enable_irq();
        return true;
    }
    return false;
}


/**
 * @brief 发送8位状态数据到K230
 * @param func 功能码
 * @param value 8位状态值
 * @return true 发送成功
 * 格式: [SOF][FUNC][VALUE][EOF]
 */
bool k230_send_status_u8(uint8_t func, uint8_t value)
{
    if (g_k230_tx_sem == NULL) {
        return false;
    }

    g_k230_tx_frame[0] = (uint8_t)RA6_TO_K230_SOF; // 帧起始符
    g_k230_tx_frame[1] = func;                      // 功能码
    g_k230_tx_frame[2] = value;                     // 状态值
    g_k230_tx_frame[3] = (uint8_t)RA6_TO_K230_EOF; // 帧结束符

    (void)xSemaphoreTake(g_k230_tx_sem, 0);

    fsp_err_t err = R_SCI_UART_Write(&robot_k230_ctrl, g_k230_tx_frame, sizeof(g_k230_tx_frame));
    if (err != FSP_SUCCESS) {
        LOG("K230 TX start failed, err=%d\r\n", (int)err);
        return false;
    }

    if (xSemaphoreTake(g_k230_tx_sem, pdMS_TO_TICKS(20)) != pdTRUE) {
        LOG("K230 TX wait complete timeout\r\n");
        return false;
    }

    return true;
}

void k230_notify_tx_complete_from_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (g_k230_tx_sem != NULL) {
        xSemaphoreGiveFromISR(g_k230_tx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
