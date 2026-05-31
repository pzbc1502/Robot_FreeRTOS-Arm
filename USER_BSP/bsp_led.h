#ifndef __BSP_LED_H_
#define __BSP_LED_H_

#include "hal_data.h"
#include <stdbool.h>
#include <stdint.h>

/* K230 补光灯软件 PWM 参数（周期建议 10ms） */
#ifndef K230_LED_PWM_PERIOD_MS
#define K230_LED_PWM_PERIOD_MS            (10u)
#endif

#ifndef K230_LED_DEFAULT_DUTY_PERCENT
#define K230_LED_DEFAULT_DUTY_PERCENT     (100u)
#endif

void LED_Init(void);
void K230_LED_Control(bool on);
void K230_LED_SetBrightness(uint8_t duty_percent);
uint8_t K230_LED_GetBrightness(void);

#define USER_LED_ON          R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_00, BSP_IO_LEVEL_LOW); // 点亮用户 LED
#define K230_LED_ON          K230_LED_Control(true)  // 开启补光 LED（按当前亮度占空比输出）
#define RED_LED_ON           R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_14, BSP_IO_LEVEL_HIGH); // 点亮红色 LED
#define BLUE_LED_ON          R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_15, BSP_IO_LEVEL_HIGH); // 点亮蓝色 LED

#define USER_LED_OFF         R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_00, BSP_IO_LEVEL_HIGH); // 熄灭用户 LED
#define K230_LED_OFF         K230_LED_Control(false) // 关闭补光 LED
#define RED_LED_OFF          R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_14, BSP_IO_LEVEL_LOW); // 熄灭红色 LED
#define BLUE_LED_OFF         R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_15, BSP_IO_LEVEL_LOW); // 熄灭蓝色 LED

#define USER_LED_TODGGLE     R_PORT4->PODR ^=1 << (BSP_IO_PORT_04_PIN_00 & 0xFF)
#define K230_LED_TODGGLE     R_PORT8->PODR ^=1 << (BSP_IO_PORT_08_PIN_01 & 0xFF)
#define RED_LED_TODGGLE      R_PORT1->PODR ^=1 << (BSP_IO_PORT_01_PIN_14 & 0xFF)
#define BLUE_LED_TODGGLE     R_PORT1->PODR ^=1 << (BSP_IO_PORT_01_PIN_15 & 0xFF)

#endif
