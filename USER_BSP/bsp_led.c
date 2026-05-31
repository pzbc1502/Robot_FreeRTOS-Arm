#include "bsp_led.h"
#include "FreeRTOS.h"
#include "timers.h"

static TimerHandle_t s_k230_led_timer = NULL;
static volatile bool s_k230_led_enable = false;
static volatile uint8_t s_k230_led_duty_percent = K230_LED_DEFAULT_DUTY_PERCENT;
static uint8_t s_k230_led_phase = 0u;

static inline void k230_led_gpio_write(bool on)
{
    (void) R_IOPORT_PinWrite(&g_ioport_ctrl,
                             BSP_IO_PORT_08_PIN_01,
                             on ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
}

static void k230_led_timer_cb(TimerHandle_t xTimer)
{
    (void) xTimer;

    if (!s_k230_led_enable)
    {
        s_k230_led_phase = 0u;
        k230_led_gpio_write(false);
        return;
    }

    uint8_t duty = s_k230_led_duty_percent;
    if (duty > 100u)
    {
        duty = 100u;
    }

    uint8_t on_ticks = (uint8_t) ((duty * K230_LED_PWM_PERIOD_MS + 99u) / 100u);
    if (on_ticks > K230_LED_PWM_PERIOD_MS)
    {
        on_ticks = K230_LED_PWM_PERIOD_MS;
    }

    k230_led_gpio_write(s_k230_led_phase < on_ticks);

    s_k230_led_phase++;
    if (s_k230_led_phase >= K230_LED_PWM_PERIOD_MS)
    {
        s_k230_led_phase = 0u;
    }
}

void K230_LED_Control(bool on)
{
    s_k230_led_enable = on;
    if (!on)
    {
        s_k230_led_phase = 0u;
        k230_led_gpio_write(false);
    }
}

void K230_LED_SetBrightness(uint8_t duty_percent)
{
    if (duty_percent > 100u)
    {
        duty_percent = 100u;
    }

    s_k230_led_duty_percent = duty_percent;
}

uint8_t K230_LED_GetBrightness(void)
{
    return s_k230_led_duty_percent;
}

void LED_Init(void)
{
    (void) R_IOPORT_Open(&g_ioport_ctrl, g_ioport.p_cfg);

    if (s_k230_led_timer == NULL)
    {
        s_k230_led_timer = xTimerCreate("k230_led",
                                        pdMS_TO_TICKS(1),
                                        pdTRUE,
                                        NULL,
                                        k230_led_timer_cb);
        if (s_k230_led_timer != NULL)
        {
            (void) xTimerStart(s_k230_led_timer, 0);
        }
    }

    K230_LED_Control(false);
}
