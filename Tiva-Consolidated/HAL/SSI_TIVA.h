/*
 * mySSI.h
 *
 *  Created on: Aug 27, 2018
 *      Author: Zhoubao Pang
 */

#ifndef SSI_TIVA_H_
#define SSI_TIVA_H_


#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"

void SSI0_Gurley_Config(void);
void SSI0_Orbis_Config(void);
void SSI0_Disable(void);    // Not Tested!!

void SSI1_Gurley_Config(void);
void SSI1_Orbis_Config(void);
void SSI1_Disable(void);

void SSI2_Config(void);
void SSI2_Disable(void);    // Not Tested!!

void SSI3_Config_SPI(void);

#endif /* SSI_TIVA_H_ */
