#include "vision_service_thread.h"
#include "jetson_vision.h"
#include "robot_target.h"
#include "robot_capture.h"
#include "bsp_laser.h"

void vision_service_thread_entry(void * pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    jetson_vision_init();
    robot_target_init();
    robot_capture_init();

    while (1)
    {
        target_obs_t obs = {0};
        robot_capture_obs_t capture_obs = {0};

        jetson_vision_process();
        obs.now_ms = HAL_GetTick();
        capture_obs.now_ms = obs.now_ms;

        uint8_t capture_action = 0u;
        uint8_t capture_point_id = 0u;
        if (jetson_get_capture_control(&capture_action, &capture_point_id))
        {
            (void)robot_capture_request(capture_action, capture_point_id);
        }

        bool target_enable = false;
        bool has_target_control = jetson_get_target_control(&target_enable);

        obs.has_vision = jetson_get_vision_error(&obs.dcx, &obs.dcy);
        obs.has_distance = jetson_get_safe_distance(&obs.distance_mm, &obs.distance_valid);
        capture_obs.has_distance = obs.has_distance;
        capture_obs.distance_mm = obs.distance_mm;
        capture_obs.distance_valid = obs.distance_valid;
        obs.fire_button = BSP_Laser_FireKey_IsPressed();
        obs.estop_active = jetson_is_unified_protocol_active() &&
                           !jetson_is_link_alive(obs.now_ms);
        capture_obs.estop_active = obs.estop_active;

        if (robot_capture_is_active())
        {
            if (has_target_control)
            {
                if (target_enable)
                {
                    (void)jetson_send_status_u8(RA6_TO_JETSON_ERROR, JETSON_ERROR_BUSY);
                }
                (void)jetson_send_status_u8(RA6_TO_JETSON_TARGET_CTRL, 0u);
            }

            robot_capture_step(&capture_obs);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (has_target_control)
        {
            bool enabled = false;
            if (target_enable)
            {
                enabled = robot_target_enable_request();
            }
            else
            {
                robot_target_disable_request();
            }
            (void)jetson_send_status_u8(RA6_TO_JETSON_TARGET_CTRL, enabled ? 1u : 0u);
        }

        robot_capture_step(&capture_obs);
        robot_target_step(&obs);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
