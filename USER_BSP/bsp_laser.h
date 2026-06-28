#ifndef BSP_LASER_H_
#define BSP_LASER_H_

#include "hal_data.h"

#ifndef LASER_PIN
#define LASER_PIN BSP_IO_PORT_00_PIN_15
#endif

void BSP_Laser_Init(void);
void BSP_Laser_On(void);
void BSP_Laser_Off(void);

#endif /* BSP_LASER_H_ */
