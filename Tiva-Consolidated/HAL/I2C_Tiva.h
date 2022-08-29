/**
 * I2C.h
 * @author: Nick Tremaroli
 * Contains all of the low-level functions required to communicate overt I2C
 */

//#include <stdint.h>
//#include <stdbool.h>
//
//#include "driverlib/gpio.h"
//#include "driverlib/pin_map.h"
//#include "inc/hw_memmap.h"
//#include "driverlib/sysctl.h"
//#include "driverlib/i2c.h"


#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"


void I2C2_Config();
void InitI2C0();

void I2C_WriteByte(uint32_t IMUBase, uint8_t I2Caddress, uint8_t regAddress, uint8_t data);
uint8_t I2C_ReadByte(uint32_t IMUBase, uint8_t I2Caddress, uint8_t regAddress);
uint8_t I2C_ReadBytes(uint32_t IMUBase, uint8_t I2Caddress, uint8_t regAddress, uint16_t length, uint8_t* data);
