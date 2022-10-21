/**
 * QEI_TIVA.h
 * @author: Nick Tremaroli
 * Contains the layout and functions regarding
 * QEI related communication of the Tiva
 */

#ifndef QEI_TIVA_H_
#define QEI_TIVA_H_

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/qei.h"

// Configure QEI Base 0
void QEIConfig0(void);

// Configure QEI Base 1
void QEIConfig1(void);


#endif /* QEI_TIVA_H_ */
