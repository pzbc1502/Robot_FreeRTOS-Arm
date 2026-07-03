#include "bsp_led.h"

void LED_Init(void)
{
    (void) R_IOPORT_Open(&g_ioport_ctrl, g_ioport.p_cfg);
}
