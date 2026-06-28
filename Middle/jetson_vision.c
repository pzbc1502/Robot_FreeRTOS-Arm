#include "jetson_vision.h"
#include "hal_data.h"
#include "bsp_uart.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* FSP generated UART names are kept unchanged to avoid regenerating FSP config. */
extern sci_uart_instance_ctrl_t robot_k230_ctrl;
extern const uart_cfg_t robot_k230_cfg;

#if defined(g_dma_k230_rx)
extern const transfer_instance_t g_dma_k230_rx;
#define JETSON_TRANSFER_INSTANCE g_dma_k230_rx
#else
extern const transfer_instance_t g_transfer_k230_rx;
#define JETSON_TRANSFER_INSTANCE g_transfer_k230_rx
#endif

typedef enum
{
    JETSON_PARSER_WAIT_SOF = 0,
    JETSON_PARSER_READ_LEN,
    JETSON_PARSER_READ_FUNC,
    JETSON_PARSER_READ_PAYLOAD,
    JETSON_PARSER_READ_CHECKSUM,
    JETSON_PARSER_READ_EOF,
} jetson_parser_state_t;

typedef struct
{
    jetson_parser_state_t state;
    uint8_t len;
    uint8_t func;
    uint8_t payload[JETSON_VISION_PAYLOAD_LEN];
    uint8_t payload_index;
    uint8_t checksum;
} jetson_parser_t;

static jetson_parser_t s_parser = { .state = JETSON_PARSER_WAIT_SOF };
static uint8_t s_rx_buffer[1024];
static uint32_t s_last_read_pos = 0u;

static int16_t s_latest_dcx = 0;
static int16_t s_latest_dcy = 0;
static volatile bool s_new_error = false;

static SemaphoreHandle_t s_tx_sem = NULL;
static uint8_t s_tx_frame[4];

static void parser_reset(void)
{
    s_parser.state = JETSON_PARSER_WAIT_SOF;
    s_parser.len = 0u;
    s_parser.func = 0u;
    s_parser.payload_index = 0u;
    s_parser.checksum = 0u;
}

static void handle_valid_error_frame(void)
{
    int16_t dcx = (int16_t)(((uint16_t)s_parser.payload[1] << 8) | (uint16_t)s_parser.payload[0]);
    int16_t dcy = (int16_t)(((uint16_t)s_parser.payload[3] << 8) | (uint16_t)s_parser.payload[2]);

    __disable_irq();
    s_latest_dcx = dcx;
    s_latest_dcy = dcy;
    s_new_error = true;
    __enable_irq();
}

static void process_byte(uint8_t byte)
{
    switch (s_parser.state)
    {
        case JETSON_PARSER_WAIT_SOF:
            if (byte == JETSON_SOF)
            {
                s_parser.state = JETSON_PARSER_READ_LEN;
                s_parser.payload_index = 0u;
                s_parser.checksum = 0u;
            }
            break;

        case JETSON_PARSER_READ_LEN:
            if (byte != JETSON_FRAME_LEN)
            {
                parser_reset();
                break;
            }
            s_parser.len = byte;
            s_parser.checksum = (uint8_t)(s_parser.checksum + byte);
            s_parser.state = JETSON_PARSER_READ_FUNC;
            break;

        case JETSON_PARSER_READ_FUNC:
            if (byte != JETSON_FUNC_VISION_ERROR)
            {
                parser_reset();
                break;
            }
            s_parser.func = byte;
            s_parser.checksum = (uint8_t)(s_parser.checksum + byte);
            s_parser.state = JETSON_PARSER_READ_PAYLOAD;
            break;

        case JETSON_PARSER_READ_PAYLOAD:
            s_parser.payload[s_parser.payload_index++] = byte;
            s_parser.checksum = (uint8_t)(s_parser.checksum + byte);
            if (s_parser.payload_index >= JETSON_VISION_PAYLOAD_LEN)
            {
                s_parser.state = JETSON_PARSER_READ_CHECKSUM;
            }
            break;

        case JETSON_PARSER_READ_CHECKSUM:
            s_parser.state = (byte == s_parser.checksum) ? JETSON_PARSER_READ_EOF : JETSON_PARSER_WAIT_SOF;
            if (s_parser.state == JETSON_PARSER_WAIT_SOF)
            {
                parser_reset();
            }
            break;

        case JETSON_PARSER_READ_EOF:
            if (byte == JETSON_EOF)
            {
                handle_valid_error_frame();
            }
            parser_reset();
            break;

        default:
            parser_reset();
            break;
    }
}

void jetson_vision_init(void)
{
    if (s_tx_sem == NULL)
    {
        s_tx_sem = xSemaphoreCreateBinary();
    }

    parser_reset();
    s_last_read_pos = 0u;

    fsp_err_t err = R_SCI_UART_Open(&robot_k230_ctrl, &robot_k230_cfg);
    if (FSP_SUCCESS != err)
    {
        LOG("Jetson UART open failed, err=%d\r\n", (int)err);
        __BKPT(0);
    }

    err = R_SCI_UART_Read(&robot_k230_ctrl, s_rx_buffer, sizeof(s_rx_buffer));
    if (FSP_SUCCESS != err)
    {
        LOG("Jetson RX start failed, err=%d\r\n", (int)err);
    }
}

void jetson_vision_process(void)
{
    uint32_t rx_buf_size = (uint32_t)sizeof(s_rx_buffer);
    transfer_properties_t props = {0};
    fsp_err_t err = JETSON_TRANSFER_INSTANCE.p_api->infoGet(JETSON_TRANSFER_INSTANCE.p_ctrl, &props);
    if (FSP_SUCCESS != err)
    {
        return;
    }

    uint32_t remaining = props.transfer_length_remaining;
    if (remaining > rx_buf_size)
    {
        remaining = rx_buf_size;
    }

    if (remaining == 0u)
    {
        uint32_t i = s_last_read_pos;
        while (i < rx_buf_size)
        {
            process_byte(s_rx_buffer[i]);
            i++;
        }

        s_last_read_pos = 0u;
        (void)R_SCI_UART_Read(&robot_k230_ctrl, s_rx_buffer, rx_buf_size);
        return;
    }

    uint32_t current_write_pos = (rx_buf_size - remaining) % rx_buf_size;
    if (current_write_pos != s_last_read_pos)
    {
        uint32_t i = s_last_read_pos;
        while (i != current_write_pos)
        {
            process_byte(s_rx_buffer[i]);
            i = (i + 1u) % rx_buf_size;
        }
        s_last_read_pos = current_write_pos;
    }
}

bool jetson_get_vision_error(int16_t *dcx, int16_t *dcy)
{
    if ((dcx == NULL) || (dcy == NULL))
    {
        return false;
    }

    if (!s_new_error)
    {
        return false;
    }

    __disable_irq();
    *dcx = s_latest_dcx;
    *dcy = s_latest_dcy;
    s_new_error = false;
    __enable_irq();
    return true;
}

bool jetson_send_status_u8(uint8_t func, uint8_t value)
{
    if (s_tx_sem == NULL)
    {
        return false;
    }

    s_tx_frame[0] = RA6_TO_JETSON_SOF;
    s_tx_frame[1] = func;
    s_tx_frame[2] = value;
    s_tx_frame[3] = RA6_TO_JETSON_EOF;

    (void)xSemaphoreTake(s_tx_sem, 0);
    fsp_err_t err = R_SCI_UART_Write(&robot_k230_ctrl, s_tx_frame, sizeof(s_tx_frame));
    if (FSP_SUCCESS != err)
    {
        LOG("Jetson TX start failed, err=%d\r\n", (int)err);
        return false;
    }

    if (xSemaphoreTake(s_tx_sem, pdMS_TO_TICKS(20)) != pdTRUE)
    {
        LOG("Jetson TX wait complete timeout\r\n");
        return false;
    }

    return true;
}

void jetson_notify_tx_complete_from_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (s_tx_sem != NULL)
    {
        xSemaphoreGiveFromISR(s_tx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
