#include "bsp_laser.h"

void BSP_Laser_Init(void)
{
    (void) R_IOPORT_PinCfg(g_ioport.p_ctrl,
                           LASER_PIN,
                           (uint32_t) IOPORT_CFG_PORT_DIRECTION_OUTPUT |
                           (uint32_t) IOPORT_CFG_PORT_OUTPUT_LOW);
    BSP_Laser_Off();
}

void BSP_Laser_On(void)
{
    (void) R_IOPORT_PinWrite(g_ioport.p_ctrl, LASER_PIN, BSP_IO_LEVEL_HIGH);
}

void BSP_Laser_Off(void)
{
    (void) R_IOPORT_PinWrite(g_ioport.p_ctrl, LASER_PIN, BSP_IO_LEVEL_LOW);
}
