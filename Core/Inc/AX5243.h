/*
 * AD5243.h
 *
 *  Created on: 29.04.2021
 *      Author: S.K.TUSUR
 */

#ifndef AD5243_H_
#define AD5243_H_

#include "main.h"


#define AX5243_CLK_PORT 		GPIOB
#define AX5243_CLK_PIN          1
#define AX5243_MISO_PORT  	    GPIOA
#define AX5243_MISO_PIN         7
#define AX5243_MOSI_PORT 	    GPIOA
#define AX5243_MOSI_PIN         6
#define AX5243_IRQ_PORT 		GPIOA
#define AX5243_IRQ_PIN          5
#define AX5243_SS_PORT 		    GPIOA
#define AX5243_SS_PIN           9
#define LED_PORT                GPIOA
#define LED_PIN 		        4


#define AX5243_delay                   2
#define AX5243_used_TIM                TIM3


#define default_shaping 1
#define default_PWR_dbm  11
#define default_f_carrier  868000000

int pwr(int base, uint32_t pwr);

void SPI_TR_24(uint16_t addr, uint8_t data);
void SPI_TR_16(uint8_t addr, uint8_t data);

void AX5243_u_delay(uint16_t delay);

void AX5243_reset();
void AX5243_init();
uint8_t AX5243_transmit(uint8_t* data, uint16_t size);
//uint8_t AX5243_st_transmit(uint8_t* data, uint16_t size, uint32_t fr, uint16_t pw);
uint8_t AX5243_set_fr(uint32_t fr);
uint8_t AX5243_set_pw(uint16_t pw);
uint8_t AX5243_set_fl(uint8_t fl);
uint8_t AX5243_set_pd(uint8_t pd);
void AX5243_rs();
#endif /* AD5243_H_ */
