#include "bsp_laser.h"
#include <stddef.h>

void BSP_Laser_Init(void)
{
    (void) R_IOPORT_PinCfg(g_ioport.p_ctrl,
                           LASER_PIN,
                           (uint32_t) IOPORT_CFG_PORT_DIRECTION_OUTPUT |
                           (uint32_t) IOPORT_CFG_PORT_OUTPUT_LOW);
    (void) R_IOPORT_PinCfg(g_ioport.p_ctrl,
                           LASER_FIRE_KEY_PIN,
                           (uint32_t) IOPORT_CFG_PORT_DIRECTION_INPUT |
                           (uint32_t) IOPORT_CFG_PULLUP_ENABLE);
    (void) R_IOPORT_PinCfg(g_ioport.p_ctrl,
                           GLOBAL_RESET_KEY_PIN,
                           (uint32_t) IOPORT_CFG_PORT_DIRECTION_INPUT |
                           (uint32_t) IOPORT_CFG_PULLUP_ENABLE);
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

bool BSP_Laser_FireKey_IsPressed(void)
{
    bsp_io_level_t level = BSP_IO_LEVEL_HIGH;
    if (FSP_SUCCESS != R_IOPORT_PinRead(g_ioport.p_ctrl, LASER_FIRE_KEY_PIN, &level))
    {
        return false;
    }

    return (level == LASER_FIRE_KEY_ACTIVE_LEVEL);
}

bool BSP_GlobalResetKey_Read(bool *pressed)
{
    if (pressed == NULL)
    {
        return false;
    }
    *pressed = false;
    bsp_io_level_t level = BSP_IO_LEVEL_HIGH;
    if (FSP_SUCCESS != R_IOPORT_PinRead(g_ioport.p_ctrl, GLOBAL_RESET_KEY_PIN, &level))
    {
        return false;
    }
    *pressed = (level == BSP_IO_LEVEL_LOW);
    return true;
}
