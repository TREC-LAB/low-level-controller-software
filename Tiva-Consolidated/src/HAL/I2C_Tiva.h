/**
 * I2C.h
 * @author: Nick Tremaroli
 * Contains the layout and functions regarding Tiva communication over I2C
 */

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

// Configure I2C1
void I2C1_Config();

// Configure I2C2
void I2C2_Config();

// Write a single byte to an I2C device
void I2C_WriteByte(uint32_t I2CBase, uint8_t I2Caddress, uint8_t regAddress, uint8_t data);

// Read a single byte from an I2C address
uint8_t I2C_ReadByte(uint32_t I2CBase, uint8_t I2Caddress, uint8_t regAddress);

// Read multiple bytes from an I2C address
void I2C_ReadBytes(uint32_t I2CBase, uint8_t I2Caddress, uint8_t regAddress, uint16_t length, uint8_t* data);
