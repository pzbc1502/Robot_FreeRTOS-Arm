#include "vision_service_thread.h"
#include "jetson_vision.h"
#include "robot_target.h"

void vision_service_thread_entry(void * pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    jetson_vision_init();
    robot_target_init();

    while (1)
    {
        target_obs_t obs = {0};
        obs.now_ms = HAL_GetTick();

        jetson_vision_process();
        obs.has_vision = jetson_get_vision_error(&obs.dcx, &obs.dcy);
        obs.fire_button = ROBOT_TARGET_FIRE_ENABLE;

        robot_target_step(&obs);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
