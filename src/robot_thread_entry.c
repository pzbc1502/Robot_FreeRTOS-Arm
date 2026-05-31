#include "robot_thread.h"
#include "bsp_led.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "robot.h"
#include "onenet.h"
#include "esp8266.h"
#include "Middle/K230_cmd.h"
#include "robot_chassis_app.h"

#include "task.h"

#include "bsp_dht11.h"


#include <stdint.h>
#include <stdio.h>
#include <string.h>


static const char *s_dev_sub_topics[] = {"/myRobot/control"};
static const char *s_dev_pub_topic = "/myRobot/status";

extern volatile uint32_t g_strawberry_picked_count;


static uint32_t g_total_ripe_found = 0;
static uint32_t g_total_unripe_found = 0;
static uint8_t last_frame_ripe = 0;
static uint8_t last_frame_unripe = 0;

/* robot_thread entry function */

/* pvParameters contains TaskHandle_t */
void robot_thread_entry(void * pvParameters)
{
	FSP_PARAMETER_NOT_USED(pvParameters);
	
	LED_Init();	
	USER_LED_ON;
	K230_LED_ON;
	BSP_UART_Init();
	LOG("BSP_UART_Init Success！！！\r\n");
	BSP_CAN_Init();
	LOG("BSP_CAN_Init Success！！！\r\n");	
	USER_LED_OFF;
	K230_LED_OFF;

    /* DHT11 init test */
    uint8_t dht_ok = (DHT11_Init() == 0) ? 1u : 0u;
    LOG("DHT11 init %s\r\n", dht_ok ? "OK" : "FAIL");

#if ESP8266_MQTT_ENABLE
    /* ESP8266 + OneNET init */
    ESP8266_Init();
	LOG("ESP8266_Init Success！！！");
    while (OneNet_DevLink())
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    OneNet_Subscribe(s_dev_sub_topics, 1);
#endif

	
	// Initialize and start all robot tasks
    robot_chassis_app_init();
	robot_init();



    /* TODO: add your own code here */
    uint32_t timeCount = 0;
    uint32_t pub_seq = 0;
    char pub_buf[256];
    uint8_t humiH = 0, humiL = 0, tempH = 0, tempL = 0;
    uint32_t ripe_cnt = 0, unripe_cnt = 0;


	while(1)
	{
#if ESP8266_MQTT_ENABLE
        unsigned char *dataPtr = ESP8266_GetIPD(3);
        if (dataPtr != NULL)
        {
            OneNet_RevPro(dataPtr);
        }
#endif


        {
            k230_strawberry_count_t count;
            if (k230_get_latest_strawberry_count(&count))
            {
                // 如果当前画面看到的草莓比上一帧多，说明发现了“新”草莓（或者识别更全了）
                // 我们把增量部分累加到总数里
                if (count.ripe > last_frame_ripe)
                {
                    g_total_ripe_found += (count.ripe - last_frame_ripe);
                }
                if (count.unripe > last_frame_unripe)
                {
                    g_total_unripe_found += (count.unripe - last_frame_unripe);
                }
                
                // 记录这一帧的数量，用于下一帧对比
                last_frame_ripe = count.ripe;
                last_frame_unripe = count.unripe;
                
                // 显示总累计数
                ripe_cnt = g_total_ripe_found;
                unripe_cnt = g_total_unripe_found;
            }


        }

#if ESP8266_MQTT_ENABLE
        timeCount++;
        if (timeCount >= 5)
        {
            timeCount = 0;
            pub_seq++;

            if (dht_ok == 0) {
                dht_ok = (DHT11_Init() == 0) ? 1u : 0u;
            }
            if (dht_ok == 1u) {
                if (DHT11_Read_Data(&humiH, &humiL, &tempH, &tempL) != 0) {
                    dht_ok = 0;
                }
            }

            uint8_t car_status = 1; // 0是关闭自主寻园 1是开启自主寻园

            snprintf(pub_buf, sizeof(pub_buf),
                     "{\"ripe\":%u,\"unripe\":%u,\"grab\":%lu,\"car\":%u,\"temp_indoor\":\"%u.%u\",\"humi_indoor\":\"%u.%u\"}",
                     (unsigned)ripe_cnt, 
					 (unsigned)unripe_cnt,
                     (unsigned long)g_strawberry_picked_count,
                     (unsigned)car_status,
                     (unsigned)tempH, (unsigned)tempL,
                     (unsigned)humiH, (unsigned)humiL);
					 
            OneNet_Publish(s_dev_pub_topic, pub_buf);

        }
#endif

		vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName)
{
    (void) xTask;
    LOG("STACK OVERFLOW: %s\r\n", (pcTaskName != NULL) ? pcTaskName : "unknown");
    taskDISABLE_INTERRUPTS();
    while (1)
    {
		vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



