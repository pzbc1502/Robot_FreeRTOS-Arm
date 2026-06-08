#include "vision_service_thread.h"

/* vision_service_thread 已裁剪：视觉采摘功能已移除，此线程保留为空任务。 */
void vision_service_thread_entry(void * pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
