/*
 * SW_interfaces.h
 *
 *  Created on: 29.04.2021
 *      Author: S.K.TUSUR
 */

#ifndef SW_interfaces_H_
#define SW_interfaces_H_

#include "main.h"

#define I2C_SDA_PORT GPIOA
#define I2C_SDA_PIN 0
#define I2C_SCL_PORT GPIOA
#define I2C_SCL_PIN 1

#define SPI_SENS_CLK_PORT 		   GPIOA
#define SPI_SENS_CLK_PIN           1
#define SPI_SENS_MISO_PORT 	       GPIOA
#define SPI_SENS_MISO_PIN 	       10
#define SPI_SENS_MOSI_PORT 	       GPIOA
#define SPI_SENS_MOSI_PIN	       0
#define SPI_SENS_SS_PORT 		   GPIOF
#define SPI_SENS_SS_PIN            1

#define used_TIM                TIM3
#define ti_delay                10            //us
#define ts_delay                2             //us

void cdelay(uint16_t ms_delay);
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_Write_Byte(uint8_t data);
uint8_t I2C_Read_Byte(uint8_t ACK);
void SPI2_Write_Byte(uint8_t addr, uint8_t data);
uint8_t SPI2_Read_Byte(uint8_t addr);

#endif /* SW_interfaces_H_ */
