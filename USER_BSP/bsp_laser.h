#ifndef BSP_LASER_H_
#define BSP_LASER_H_

#include <stdbool.h>
#include "hal_data.h"

#ifndef LASER_PIN
#define LASER_PIN BSP_IO_PORT_08_PIN_01
#endif

#ifndef LASER_FIRE_KEY_PIN
/* P002 -- switch -- GND; the internal pull-up makes an open switch inactive. */
#define LASER_FIRE_KEY_PIN BSP_IO_PORT_00_PIN_02
#endif

#ifndef LASER_FIRE_KEY_ACTIVE_LEVEL
#define LASER_FIRE_KEY_ACTIVE_LEVEL BSP_IO_LEVEL_LOW
#endif

#define GLOBAL_RESET_KEY_PIN BSP_IO_PORT_00_PIN_00

void BSP_Laser_Init(void);
void BSP_Laser_On(void);
void BSP_Laser_Off(void);
bool BSP_Laser_FireKey_IsPressed(void);
/* Returns sample validity; an IO error must not count as a released reset key. */
bool BSP_GlobalResetKey_Read(bool *pressed);

#endif /* BSP_LASER_H_ */
