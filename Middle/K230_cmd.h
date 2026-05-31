#ifndef K230_CMD_H_
#define K230_CMD_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * K230 -> RA6M5 (RA6M5接收端) 帧格式 (三组不同的SOF/EOF组合):
 *   [SOF][LEN][FUNC][DATA...][CHECKSUM][EOF]
 * - LEN: FUNC(1字节) + DATA(N字节) 的总长度
 * - CHECKSUM: (LEN + FUNC + DATA...) 所有字节的8位无符号和（溢出自动截断）
 * - "判断帧" 使用: SOF=0xAA, EOF=0xBB, DATA=1字节 (0x00=不完整/0x01=完整)
 * - "坐标帧" 使用: SOF=0xFF, EOF=0xFE, DATA=4字节 (Cx,Cy 两个int16坐标值)
 * - "草莓数量帧" 使用: SOF=0xAB, EOF=0xBA, DATA=2字节 (成熟数, 未成熟数)
 */ 


/* --- 协议常量 (K230 -> RA6M5) --- */
/* --- (K230 -> RA6M5)判断帧和数据帧的帧头与帧尾定义 --- */

// 判断帧标识（用于草莓完整性/可采摘状态）
#define K230_SOF_JUDGE        0xAAu  // 帧起始符(Start of Frame)
#define K230_EOF_JUDGE        0xBBu  // 帧结束符(End of Frame)

// 坐标数据帧标识
#define K230_SOF_COORD        0xFFu  // 帧起始符
#define K230_EOF_COORD        0xFEu  // 帧结束符

// 草莓数量帧标识
#define K230_SOF_COUNT        0xABu  // 帧起始符
#define K230_EOF_COUNT        0xBAu  // 帧结束符

// 最大有效负载长度（安全限制）
#define MAX_PAYLOAD_SIZE      32u


/* --- 功能码定义 (K230 -> RA6M5) --- */
#define K230_FUNC_STRAWBERRY_INTEGRITY   0x02u  // 画面是否有草莓
                                                /* DATA[0]=0x00:没有草莓, 0x01:有的兄弟，有的 */
                                                
#define K230_FUNC_COORDINATE_DATA        0x03u  // 坐标数据
                                                /* DATA[0..3]=Cx,Cy 两个int16坐标值(像素中心偏移量) */
                                                
#define K230_FUNC_HARVESTABLE            0x04u  // 可采摘状态
                                                /* DATA[0]=0x00:不可采摘, 0x01:可采摘 */
#define K230_FUNC_STRAWBERRY_COUNT       0x06u  // 草莓成熟/未成熟数量
                                                /* DATA[0]=成熟数, DATA[1]=未成熟数 */

/* --- 功能码定义 (RA6M5 -> K230) --- */

/* --- (RA6M5 -> K230)返回帧的帧头与帧尾定义 --- */
#define RA6_TO_K230_SOF   0xCCu  // RA6M5发送帧起始符
#define RA6_TO_K230_EOF   0xDDu  // RA6M5发送帧结束符

/* RA6M5 -> K230 功能码 */
#define RA6_FUNC_AT_PRESTART             0x01u  // 预开始位置确认
                                                /* DATA[0]=0x01:已到达预开始位置 */

/*
 * 坐标数据字节序配置 (Cx/Cy int16类型)
 * - 大端模式(BIG)   : [x高字节, x低字节, y高字节, y低字节]
 * - 小端模式(LITTLE): [x低字节, x高字节, y低字节, y高字节]
 */
#define K230_COORD_ENDIAN_BIG     1u    // 大端模式
#define K230_COORD_ENDIAN_LITTLE  0u    // 小端模式
#ifndef K230_COORD_ENDIAN
#define K230_COORD_ENDIAN K230_COORD_ENDIAN_LITTLE  // 默认使用小端模式
#endif

/**
 * @brief 2D坐标点结构体 (来自K230的Cx,Cy坐标)
 * 坐标单位：像素中心偏移量
 * 原点位置：图像中心（正右/正下为正方向）
 */
typedef struct
{
    int16_t x;  // X轴坐标（水平方向）
    int16_t y;  // Y轴坐标（垂直方向）
} k230_point_t;

/**
 * @brief 草莓成熟/未成熟数量结构体
 */
typedef struct
{
    uint8_t ripe;     // 成熟数量
    uint8_t unripe;   // 未成熟数量
} k230_strawberry_count_t;

/**
 * @brief 通用解码帧结构 (K230 -> RA6M5)
 * 存储完整解析后的帧数据
 */
typedef struct
{
    uint8_t sof;           // 帧起始符 (K230_SOF_*)
    uint8_t func;          // 功能码 (K230_FUNC_*)
    uint8_t payload_len;   // 有效负载长度（字节）
    uint8_t payload[MAX_PAYLOAD_SIZE];  // 有效负载数据缓冲区
} k230_frame_t;


/* --- 协议解析状态机 --- */
/**
 * @brief 协议解析状态机枚举
 * 严格遵循状态转换顺序，确保协议健壮性
 */
typedef enum {
    /**
     * @brief 等待帧起始符(SOF)状态
     * - 持续扫描输入字节流
     * - 检测到K230_SOF_COORD(0xFF)、K230_SOF_JUDGE(0xAA)或K230_SOF_COUNT(0xAB)时转换到下一状态
     * - 任何其他字节将被忽略
     */
    PARSER_STATE_WAIT_FOR_SOF,

    
    /**
     * @brief 读取帧长度(LEN)状态
     * - 接收1字节长度值
     * - 验证长度有效性：
     *   • 不能为0（至少包含1字节功能码）
     *   • 不能超过MAX_PAYLOAD_SIZE+1（功能码+最大负载）
     * - 无效长度将重置状态机
     * - 有效长度则累加到校验和，进入下一状态
     */
    PARSER_STATE_READ_LEN,
    
    /**
     * @brief 读取功能码(FUNC)状态
     * - 接收1字节功能码
     * - 将功能码累加到校验和
     * - 根据LEN值决定下一状态：
     *   • LEN>1：存在有效负载，进入PAYLOAD状态
     *   • LEN=1：无有效负载，直接进入CHECKSUM状态
     */
    PARSER_STATE_READ_FUNC,
    
    /**
     * @brief 读取有效负载(PAYLOAD)状态
     * - 循环接收(LEN-1)字节数据（功能码后的所有数据）
     * - 每个字节存入payload_buffer并累加到校验和
     * - 当接收字节数达到(LEN-1)时，进入CHECKSUM状态
     */
    PARSER_STATE_READ_PAYLOAD,
    
    /**
     * @brief 读取校验和(CHECKSUM)状态
     * - 接收1字节校验和
     * - 与本地计算的calculated_checksum比较
     * - 匹配成功：进入EOF验证状态
     * - 匹配失败：重置整个状态机（丢弃当前帧）
     */
    PARSER_STATE_READ_CHECKSUM,
    
    /**
     * @brief 读取帧结束符(EOF)状态
     * - 接收1字节结束符
     * - 验证是否匹配当前帧类型对应的EOF：
     *   • 坐标帧：K230_EOF_COORD(0xFE)
     *   • 判断帧：K230_EOF_JUDGE(0xBB)
     *   • 草莓数量帧：K230_EOF_COUNT(0xBA)

     * - 验证成功：调用handle_valid_frame()处理完整帧
     * - 无论成功与否，最终都会重置状态机（准备接收新帧）
     */
    PARSER_STATE_READ_EOF

} parser_state_t;

/**
 * @brief 协议解析器状态结构体
 * 维护协议解析过程中的所有中间状态
 */
typedef struct {
    parser_state_t state;            				// 当前解析状态（见parser_state_t枚举）
    uint8_t sof;                     				// 当前帧的SOF值（用于后续验证）
    uint8_t eof;                     				// 当前帧期望的EOF值（根据SOF确定）
    uint8_t len;                     				// 帧长度值（FUNC+PAYLOAD总字节数）
    uint8_t func;                    				// 功能码
    uint8_t payload_buffer[MAX_PAYLOAD_SIZE];  		// 有效负载临时缓冲区
    uint8_t payload_index;            				// 当前payload写入位置索引
    uint8_t calculated_checksum;      				// 动态计算的校验和
} protocol_parser_t;

/* --- 公共API接口 --- */
/**
 * @brief 初始化K230通信协议
 * - 配置UART外设
 * - 启动DMA/DTC环形缓冲区接收
 * - 初始化协议解析状态机
 */
void k230_protocol_init(void);

/**
 * @brief 处理K230协议数据
 * - 从DMA/DTC环形缓冲区获取新数据
 * - 调用状态机逐字节解析
 * - 解析成功时更新全局数据标志
 * @note 应在主循环中定期调用（建议1-10ms周期）
 */
void k230_protocol_process(void);

/* 便捷接口：获取最新坐标数据 */
/**
 * @brief 获取最新解析的坐标点
 * @param point 指向坐标结构的指针（输出参数）
 * @return true 有新坐标数据，false 无新数据
 * @note 线程安全：通过关中断保护共享数据
 */
bool k230_get_latest_coords(k230_point_t *point);

/* 通用接口：获取最新完整帧 */
/**
 * @brief 获取最新解析的完整帧
 * @param frame 指向帧结构的指针（输出参数）
 * @return true 有新帧数据，false 无新数据
 * @note 线程安全：通过关中断保护共享数据
 */
bool k230_get_latest_frame(k230_frame_t *frame);

/**
 * @brief 获取最新可采摘状态(0x04)
 * @param value 输出0/1状态
 * @return true 有新数据，false 无新数据
 */
bool k230_get_latest_harvestable(uint8_t *value);

/**
 * @brief 获取最新草莓成熟/未成熟数量
 * @param count 指向数量结构体的指针（输出参数）
 * @return true 有新数据，false 无新数据
 */
bool k230_get_latest_strawberry_count(k230_strawberry_count_t *count);

/* RA6M5 -> K230 通信接口 */
/**
 * @brief 发送8位状态数据到K230
 * @param func 功能码（RA6_FUNC_*）
 * @param value 8位状态值
 * @return true 发送成功（实际异步发送）
 * @note 帧格式: [SOF=0xCC][FUNC][VALUE][EOF=0xDD]
 */
bool k230_send_status_u8(uint8_t func, uint8_t value);
void k230_notify_tx_complete_from_isr(void);

#endif /* K230_CMD_H_ */



