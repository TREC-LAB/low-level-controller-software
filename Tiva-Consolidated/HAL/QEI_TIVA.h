/*
 * myQEI.h
 *
 *  Created on: Aug 27, 2018
 *      Author: Zhoubao Pang
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

extern void QEIConfig0(void);
extern void QEIConfig1(void);


#endif /* QEI_TIVA_H_ */
