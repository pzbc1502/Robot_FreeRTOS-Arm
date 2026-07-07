#ifndef ROBOT_CAPTURE_H_
#define ROBOT_CAPTURE_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t now_ms;
    bool has_distance;
    bool distance_valid;
    uint16_t distance_mm;
    bool estop_active;
    bool limit_triggered;
} robot_capture_obs_t;

void robot_capture_init(void);
bool robot_capture_request(uint8_t action, uint8_t point_id);
void robot_capture_step(const robot_capture_obs_t *obs);
bool robot_capture_is_active(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CAPTURE_H_ */
