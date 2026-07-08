#ifndef JETSON_VISION_H_
#define JETSON_VISION_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define JETSON_SOF                 0xFFu
#define JETSON_EOF                 0xFEu
#define JETSON_FRAME_LEN           0x05u
#define JETSON_FUNC_VISION_ERROR   0x03u
#define JETSON_VISION_PAYLOAD_LEN  4u

#define JETSON_CTRL_SOF            0xAAu
#define JETSON_CTRL_EOF            0xBBu
#define JETSON_FUNC_TARGET_CTRL    0x01u

#define JETSON_UNIFIED_SOF0        0xA5u
#define JETSON_UNIFIED_SOF1        0x5Au
#define JETSON_UNIFIED_VERSION     0x01u
#define JETSON_UNIFIED_MAX_PAYLOAD 32u

#define JETSON_MSG_HEARTBEAT       0x01u
#define JETSON_MSG_TARGET_CTRL     0x02u
#define JETSON_MSG_VISION_ERROR    0x03u
#define JETSON_MSG_CAPTURE_CTRL    0x04u
#define JETSON_MSG_SAFE_DISTANCE   0x05u
#define JETSON_MSG_STATUS          0x81u
#define JETSON_MSG_ERROR           0xFEu

#define JETSON_HEARTBEAT_TIMEOUT_MS 600u

#define RA6_TO_JETSON_SOF          0xCCu
#define RA6_TO_JETSON_EOF          0xDDu
#define RA6_TO_JETSON_READY        0x01u
#define RA6_TO_JETSON_ALIGN_DONE   0x02u
#define RA6_TO_JETSON_OUTPUT       0x03u
#define RA6_TO_JETSON_TARGET_CTRL  0x04u
#define RA6_TO_JETSON_SAFE_DISTANCE 0x06u
#define RA6_TO_JETSON_CAPTURE_POINT 0x10u
#define RA6_TO_JETSON_CAPTURE_DONE  0x11u
#define RA6_TO_JETSON_TARGET_PRESTART 0x12u
#define RA6_TO_JETSON_ERROR        0xFEu

#define JETSON_CAPTURE_ACTION_GOTO   0x01u
#define JETSON_CAPTURE_ACTION_FINISH 0x02u
#define JETSON_CAPTURE_ACTION_SELECT 0x03u
#define JETSON_CAPTURE_ACTION_CURRENT 0x04u

#define JETSON_ERROR_INVALID_PARAM 0x03u
#define JETSON_ERROR_BUSY          0x05u
#define JETSON_ERROR_HEARTBEAT_TIMEOUT 0x07u
#define JETSON_ERROR_SAFETY        0x08u
#define JETSON_ERROR_SAFE_DISTANCE_TOO_CLOSE 0x09u

void jetson_vision_init(void);
void jetson_vision_process(void);
bool jetson_get_vision_error(int16_t *dcx, int16_t *dcy);
bool jetson_get_target_control(bool *enable);
bool jetson_get_capture_control(uint8_t *action, uint8_t *point_id);
bool jetson_get_safe_distance(uint16_t *distance_mm, bool *valid);
bool jetson_is_unified_protocol_active(void);
bool jetson_is_link_alive(uint32_t now_ms);
bool jetson_send_status_u8(uint8_t func, uint8_t value);
void jetson_notify_tx_complete_from_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* JETSON_VISION_H_ */
