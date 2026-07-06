#include "vision_service_thread.h"
#include "jetson_vision.h"
#include "robot_target.h"
#include "bsp_laser.h"

void vision_service_thread_entry(void * pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    jetson_vision_init();
    robot_target_init();

    while (1)
    {
        target_obs_t obs = {0};

        jetson_vision_process();
        obs.now_ms = HAL_GetTick();

        bool target_enable = false;
        if (jetson_get_target_control(&target_enable))
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

        obs.has_vision = jetson_get_vision_error(&obs.dcx, &obs.dcy);
        obs.fire_button = BSP_Laser_FireKey_IsPressed();
        obs.estop_active = jetson_is_unified_protocol_active() &&
                           !jetson_is_link_alive(obs.now_ms);

        robot_target_step(&obs);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
