/**
 * SSI_TIVA.h
 * @author: Nick Tremaroli
 * Contains the layout and functions regarding
 * SSI related communication of the Tiva
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

// Enable SSI0 for a Gurley Encoder
void SSI0_Gurley_Config(void);

// Enable SSI0 for an Orbis Encoder
void SSI0_Orbis_Config(void);

// Disable SSI0
// TODO: needs to be tested
void SSI0_Disable(void);

// Enable SSI1 for a Gurley Encoder
void SSI1_Gurley_Config(void);

// Enable SSI1 for an Orbis Encoder
void SSI1_Orbis_Config(void);

// Disable SSI1
// TODO: needs to be tested
void SSI1_Disable(void);

// Generally Configure SSI2
void SSI2_Config(void);

//// Disable SSI2
//// TODO: needs to be tested
void SSI2_Disable(void);

// Enable SSI3 for SPI communication
void SSI3_Config_SPI(void);

#endif /* SSI_TIVA_H_ */
