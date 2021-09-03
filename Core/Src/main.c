/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "AX5243.h"
#include "SW_interfaces.h"
#include "encrypt/OpenUNBEncrypterLL.h"
#include "encrypt/OpenUNBConsts.h"
#include "polar/OpenUNBEncoderLL.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define FLASH_FKEY1 0x45670123
#define FLASH_FKEY2 0xCDEF89AB

#define FLASH_CONST_ADDR	(0x8008000 - 0x200) // last 512 bytes
#define FLASH_Na			FLASH_CONST_ADDR
#define FLASH_K0			(FLASH_Na + 4)
#define FLASH_DEV_ID_SIZE	(FLASH_K0 + 64)
#define FLASH_DEV_ID		(FLASH_DEV_ID_SIZE + 2)

const uint32_t PREAMB = 0x97157A6F;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t u2_rx_buff[500];
uint8_t tx_buff[256];
uint8_t error_flag = 0;
uint16_t RX_STR_Size;
uint32_t f_carrier;
uint16_t PWR_dbm;
uint8_t shaping_filter;
uint8_t p_d;
uint16_t RX_Data_Size;
uint8_t Dev_addr, Reg_addr, Reg_data;
uint8_t data_after_preapare[64];
uint32_t msTicks;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int preapare_msg_16bit(struct encrypt_data_t *initData, uint8_t *in,
		uint8_t *out);
int preapare_msg_48bit(struct encrypt_data_t *initData, uint8_t *in,
		uint8_t *out);
void preapare_msg_activate(struct encrypt_data_t *initData, uint8_t *out);

void USART_TX(uint8_t *dt, uint16_t sz);
void USART_TX_Str(char *string);
uint16_t USART_RX_Str(void);
uint32_t AsciiToDec(uint8_t *s, uint8_t len, uint8_t number);
uint8_t AsciiToHex(uint8_t s_h, uint8_t s_l);
uint16_t Ascii2ToHex(uint8_t *s, uint16_t len, uint8_t number);
void USART_TX_h_VAL(char *string, uint8_t val);
void USART_TX_d_VAL(char *string, uint32_t val, uint8_t len);

void FLASH_Unlock(void) {
	if ((FLASH->CR & FLASH_CR_LOCK) != RESET) {
		FLASH->KEYR = FLASH_FKEY1;
		FLASH->KEYR = FLASH_FKEY2;
	}
}

void FLASH_Lock(void) {
	FLASH->CR |= FLASH_CR_LOCK;
}

void FLASH_ClearPage(uint32_t Page_Address) {
	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR = Page_Address;
	FLASH->CR |= FLASH_CR_STRT;
	while ((FLASH->SR & FLASH_SR_BSY))
		;
	FLASH->CR &= ~FLASH_CR_PER;
}

uint16_t FLASH_Read(uint32_t Address) {
	return (*(__IO uint16_t*) Address);
}

void FLASH_Write(uint32_t Address, uint16_t Data) {

	FLASH->CR |= FLASH_CR_PG;
	while ((FLASH->SR & FLASH_SR_BSY))
		;
	*(__IO uint16_t*) Address = Data;
	while ((FLASH->SR & FLASH_SR_BSY))
		;
	FLASH->CR &= ~(FLASH_CR_PG);

}

void FLASH_Read_Data(struct encrypt_data_t *data) {
	memcpy(data, FLASH_CONST_ADDR, sizeof(struct encrypt_data_t));
}

void FLASH_Write_Data(struct encrypt_data_t *data) {
	FLASH_Unlock();
	FLASH_ClearPage(FLASH_CONST_ADDR);
	for (int i = 0; i < sizeof(struct encrypt_data_t) / 2; i++) {
		FLASH_Write(FLASH_CONST_ADDR + i * 2, ((uint16_t*) data)[i]);
	}
	FLASH_Lock();
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	/* System interrupt init*/

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */


	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	LL_TIM_EnableCounter(TIM3);
	LL_TIM_EnableCounter(TIM2);
	//(TIM2->CNT) = (0);
	LL_SYSTICK_EnableIT();
	// OpenUNB device

	struct encrypt_data_t initData;

	if (FLASH_Read(FLASH_CONST_ADDR) != 0xFFFF) {
		FLASH_Read_Data(&initData);
	} else {

		for (int i = 0; i < 16; i++) {
			initData.DevID[i] = rand() & 0xFF;
		}

		for (int i = 0; i < sizeof(initData.K0); i++) {
			initData.K0[i] = rand() & 0xFF;
		}

		initData.Na = 0;
	}

	initData.Na++;
	FLASH_Write_Data(&initData);

	initEncrypter(&initData);

	//encodeActivateMsg(&initData, act_msg, LL_GetTi);

	for (int i = 0; i < 10; i++) {                               //Indication
		LED_PORT->BSRR = (1 << LED_PIN);
		LL_mDelay(100);
		LED_PORT->BRR = (1 << LED_PIN);
		LL_mDelay(100);
	}

	f_carrier = default_f_carrier;
	PWR_dbm = default_PWR_dbm;
	shaping_filter = default_shaping;

	AX5243_init();
	USART_TX_Str("TRANSMITTER READY\n\r");

	USART_TX_h_VAL("Na: ", initData.Na);

	USART_TX_Str("DevID: ");
	for (int i = 0; i < 16; i++) {
		USART_TX_h_VAL_without_text(initData.DevID[i]);
	}
	USART_TX_Str("\n\r");

	USART_TX_Str("K0: ");
	for (int i = 0; i < sizeof(initData.K0); i++) {
		USART_TX_h_VAL_without_text(initData.K0[i]);
	}
	USART_TX_Str("\n\r");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		RX_STR_Size = USART_RX_Str();

		if ((u2_rx_buff[0] == 'A') && (u2_rx_buff[1] == 'T')) {
			if (RX_STR_Size == 2) {
				error_flag = 0;
			} else {
				error_flag = 1;
			}
			/*
			 } else if ((u2_rx_buff[0] == 'F') && (u2_rx_buff[1] == 'R')) {
			 if (RX_STR_Size == 11) {
			 f_carrier = AsciiToDec(u2_rx_buff, 9, 3);
			 if (error_flag == 0)
			 error_flag = (AX5243_set_fr(f_carrier));
			 } else {
			 error_flag = 1;
			 }
			 */
		} else if ((u2_rx_buff[0] == 'P') && (u2_rx_buff[1] == 'W')) {
			if (RX_STR_Size == 4) {
				PWR_dbm = AsciiToDec(u2_rx_buff, 2, 3);
				if (error_flag == 0)
					error_flag = (AX5243_set_pw(PWR_dbm));
			} else {
				error_flag = 1;
			}
			/*
			 } else if ((u2_rx_buff[0] == 'F') && (u2_rx_buff[1] == 'L')) {
			 if (RX_STR_Size == 3) {
			 shaping_filter = AsciiToDec(u2_rx_buff, 1, 3);
			 if (error_flag == 0)
			 error_flag = (AX5243_set_fl(shaping_filter));
			 } else {
			 error_flag = 1;
			 }
			 */
		} else if ((u2_rx_buff[0] == 'P') && (u2_rx_buff[1] == 'D')) {
			if (RX_STR_Size == 3) {
				p_d = AsciiToDec(u2_rx_buff, 1, 3);
				if (error_flag == 0)
					error_flag = (AX5243_set_pd(p_d));
			} else {
				error_flag = 1;
			}

		} /*else if ((u2_rx_buff[0] == 'T') && (u2_rx_buff[1] == 'R')) {
			RX_Data_Size = Ascii2ToHex(u2_rx_buff, RX_STR_Size, 3);
			if (error_flag == 0)
				error_flag = (AX5243_transmit(tx_buff, RX_Data_Size));
		} else if ((u2_rx_buff[0] == 'I') && (u2_rx_buff[1] == 'R')) { // IR760D
			if (RX_STR_Size == 6) {
				Dev_addr = AsciiToHex(u2_rx_buff[2], u2_rx_buff[3]);
				Reg_addr = AsciiToHex(u2_rx_buff[4], u2_rx_buff[5]);
				if (error_flag == 0) {
					I2C_Start();
					I2C_Write_Byte(Dev_addr << 1);
					I2C_Write_Byte(Reg_addr);
					I2C_Start();
					I2C_Write_Byte(Dev_addr << 1 | 0x1);
					Reg_data = I2C_Read_Byte(1);
					I2C_Stop();
					USART_TX_h_VAL("Reg_data:", Reg_data);
				}
			} else {
				error_flag = 1;
			}
		}

		/*
		 else if ((u2_rx_buff[0] == 'I') && (u2_rx_buff[1] == 'W')) { // IW760D11
		 if (RX_STR_Size == 8) {
		 Dev_addr = AsciiToHex(u2_rx_buff[2], u2_rx_buff[3]);
		 Reg_addr = AsciiToHex(u2_rx_buff[4], u2_rx_buff[5]);
		 Reg_data = AsciiToHex(u2_rx_buff[6], u2_rx_buff[7]);
		 if (error_flag == 0) {
		 I2C_Start();
		 I2C_Write_Byte(Dev_addr << 1);
		 I2C_Write_Byte(Reg_addr);
		 I2C_Write_Byte(Reg_data);
		 I2C_Stop();
		 }
		 } else {
		 error_flag = 1;
		 }
		 }


		else if ((u2_rx_buff[0] == 'S') && (u2_rx_buff[1] == 'R')) {    // SR0D
			if (RX_STR_Size == 4) {
				Reg_addr = AsciiToHex(u2_rx_buff[2], u2_rx_buff[3]);
				if (error_flag == 0) {
					Reg_data = SPI2_Read_Byte(Reg_addr);
					USART_TX_h_VAL("Reg_data:", Reg_data);
				}
			} else {
				error_flag = 1;
			}
		}*/

		else if ((u2_rx_buff[0] == 'S') && (u2_rx_buff[1] == 'W')) {   // SW0D11
			if (RX_STR_Size == 6) {
				Reg_addr = AsciiToHex(u2_rx_buff[2], u2_rx_buff[3]);
				Reg_data = AsciiToHex(u2_rx_buff[4], u2_rx_buff[5]);
				if (error_flag == 0) {
					SPI2_Write_Byte(Reg_addr, Reg_data);
				}
			} else {
				error_flag = 1;
			}
		}

		else if ((u2_rx_buff[0] == 'N') && (u2_rx_buff[1] == 'T')) {   // NT0201
			if (RX_STR_Size == 6 || RX_STR_Size == 14) {
				RX_Data_Size = Ascii2ToHex(u2_rx_buff, RX_STR_Size, 3);
				if (RX_Data_Size == 2 && error_flag == 0) {

					uint8_t tmp = tx_buff[0];
					tx_buff[0] = tx_buff[1];
					tx_buff[1] = tmp;

					error_flag = preapare_msg_16bit(&initData, tx_buff,
							data_after_preapare);
					if (error_flag != 0) {
						USART_TX_Str("Wait 1 min\n\r");
						error_flag = 1;
					} else {
						error_flag = (AX5243_transmit(data_after_preapare, 20));
					}
				} else if (RX_Data_Size == 6 && error_flag == 0) {

					for (int i = 0; i < 3; i++) {
						uint8_t tmp = tx_buff[i];
						tx_buff[i] = tx_buff[5 - i];
						tx_buff[5 - i] = tmp;
					}

					error_flag = preapare_msg_48bit(&initData, tx_buff,
							data_after_preapare);
					if (error_flag != 0) {
						USART_TX_Str("Wait 1 min\n\r");
						error_flag = 1;
					} else {
						error_flag = (AX5243_transmit(data_after_preapare, 28));
					}
				}
			} else {
				error_flag = 1;
			}
		}

		else if ((u2_rx_buff[0] == 'A') && (u2_rx_buff[1] == 'M')) {    // AM
			if (RX_STR_Size == 2) {
				for (int i = 0; i < MAX_PKT_TX_NUM && error_flag == 0; i++) {
					preapare_msg_activate(&initData, data_after_preapare);
					error_flag = (AX5243_transmit(data_after_preapare, 20));

					LL_mDelay(500);
				}
			} else {
				error_flag = 1;
			}
		}

		/*
		 else if ((u2_rx_buff[0] == 'N') && (u2_rx_buff[1] == 'I')) {   // NI760D
		 if (RX_STR_Size == 6) {
		 Dev_addr = AsciiToHex(u2_rx_buff[2], u2_rx_buff[3]);
		 Reg_addr = AsciiToHex(u2_rx_buff[4], u2_rx_buff[5]);
		 if (error_flag == 0) {
		 I2C_Start();
		 I2C_Write_Byte(Dev_addr << 1);
		 I2C_Write_Byte(Reg_addr);
		 I2C_Start();
		 I2C_Write_Byte(Dev_addr << 1 | 0x1);
		 Reg_data = I2C_Read_Byte(1);
		 I2C_Stop();
		 tx_buff[0] = Reg_addr;
		 tx_buff[1] = Reg_data;
		 error_flag = preapare_msg_16bit(&initData, tx_buff,
		 data_after_preapare);
		 if (error_flag != 0) {
		 error_flag = 1;
		 } else {
		 error_flag = (AX5243_transmit(data_after_preapare, 20));
		 }
		 }
		 } else {
		 error_flag = 1;
		 }
		 }

		 /*
		 else if ((u2_rx_buff[0] == 'N') && (u2_rx_buff[1] == 'S')) {    // NS0D
		 if (RX_STR_Size == 4) {
		 Reg_addr = AsciiToHex(u2_rx_buff[2], u2_rx_buff[3]);
		 if (error_flag == 0) {
		 Reg_data = SPI2_Read_Byte(Reg_addr);
		 tx_buff[0] = Reg_addr;
		 tx_buff[1] = Reg_data;
		 error_flag = preapare_msg_16bit(&initData, tx_buff,
		 data_after_preapare);
		 if (error_flag != 0) {
		 error_flag = 1;
		 } else {
		 error_flag = (AX5243_transmit(data_after_preapare, 20));
		 }
		 }
		 } else {
		 error_flag = 1;
		 }
		 }

		 /*
		 else if ((u2_rx_buff[0] == 'S') && (u2_rx_buff[1] == 'V')) {
		 if (RX_STR_Size == 2) {
		 USART_TX_Str("Software version:2.0 \n\r");
		 } else {
		 error_flag = 1;
		 }
		 } */
		else if ((u2_rx_buff[0] == 'T') && (u2_rx_buff[1] == 'S')) {
			if (RX_STR_Size == 2) {
				USART_TX_Str("Transmitter settings\n\r");
				USART_TX_d_VAL("carrier frequency:", f_carrier, 9);
				USART_TX_d_VAL("transmit power:", PWR_dbm, 2);
				USART_TX_d_VAL("shaping filter:", shaping_filter, 1);
			} else {
				error_flag = 1;
			}
		} else if ((u2_rx_buff[0] == 'R') && (u2_rx_buff[1] == 'S')) {
			if (RX_STR_Size == 2) {
				AX5243_rs();
			} else {
				error_flag = 1;
			}

		} else if ((u2_rx_buff[0] == 'D') && (u2_rx_buff[1] == 'A')) {
			if (RX_STR_Size > 2 && RX_STR_Size == 32) {

				for (int i = 0; i < 16; i++) {
					initData.DevID[i] = AsciiToHex(u2_rx_buff[2 + i * 2],
							u2_rx_buff[2 + i * 2 + 1]);
				}

				FLASH_Write_Data(&initData);
			} else {
				error_flag = 1;
			}
		} else if ((u2_rx_buff[0] == 'K') && (u2_rx_buff[1] == '0')) {
			if (RX_STR_Size == 2) {
				for (int i = 0; i < sizeof(initData.K0); i++) {
					initData.K0[i] = rand() & 0xFF;
				}

				USART_TX_Str("K0: ");
				for (int i = 0; i < sizeof(initData.K0); i++) {
					USART_TX_h_VAL_without_text(initData.K0[i]);
				}
				USART_TX_Str("\n\r");

				FLASH_Write_Data(&initData);
			} else {
				error_flag = 1;
			}

		} else if ((u2_rx_buff[0] == 'E') && (u2_rx_buff[1] == 'F')) {
			if (RX_STR_Size == 2) {
				FLASH_Unlock();
				FLASH_ClearPage(FLASH_CONST_ADDR);
				FLASH_Lock();
			} else {
				error_flag = 1;
			}

		} else if ((u2_rx_buff[0] == 'S') && (u2_rx_buff[1] == 'C')) {
			if (RX_STR_Size == 2) {

				USART_TX_Str("Supported Commands:\n\r");
				USART_TX_Str("AT-communication test,\n\r");
				//USART_TX_Str("FRxxxxxxxxx-carrier frequency,\n\r");
				USART_TX_Str("PWxx-transmitter power,\n\r");
				USART_TX_Str("SV-software version,\n\r");
				USART_TX_Str("TRzz...z-raw data transfer,\n\r");
				USART_TX_Str("TS-transmitter settings,\n\r");
				USART_TX_Str("RS-module reboot,\n\r");
				USART_TX_Str("SC-command list,\n\r");
				USART_TX_Str("PDx-Phase direction.\n\r");
				USART_TX_Str("DAxx..xx - device address.\n\r");
				USART_TX_Str("K0 - generate new device key.\n\r");
				USART_TX_Str("EF - erase flash data device.\n\r");
				//USART_TX_Str(
				//		"FLx- amplitude shape filter at phase shift (0-OFF, 1-ON),\n\r");
				//USART_TX_Str(
				//		"IRxxyy - external sensor reading by I2C. xx - sensor address, yy - register address,\n\r");
				//USART_TX_Str(
				//		"IWxxyyzz - Writing to an external sensor by I2C. xx-sensor address, yy-register address, zz-value,\n\r");
				//USART_TX_Str(
				//		"SRxx - Reading an external sensor by SPI. xx- register address,\n\r");
				//USART_TX_Str(
				//		"SWxxyy - Write to external sensor by SPI. xx- register address, yy- register value,\n\r");
				USART_TX_Str(
						"NTxx..x- Data transfer by OpenUNB protocol (data length 2 or 6 bytes),\n\r");
				USART_TX_Str(
						"AM- transmission of the OpenUNB activation message,\n\r");
				USART_TX_Str(
						"NIxxyy - Reading external sensor by I2C and send by OpenUNB protocol. xx - sensor address, yy - register address,\n\r");
				USART_TX_Str(
						"NSxx - Reading external sensor by SPI and sending by OpenUNB protocol. xx- register address.\n\r");

			} else {
				error_flag = 1;
			}
		}

		else {
			error_flag = 1;
		}
		if (!error_flag) {
			USART_TX_Str("OK\n\r");
			error_flag = 0;
		} else {
			USART_TX_Str("ERROR\n\r");
			error_flag = 0;
		}
		LL_mDelay(1);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0) {
	}
	LL_RCC_HSE_EnableBypass();
	LL_RCC_HSE_Enable();

	/* Wait till HSE is ready */
	while (LL_RCC_HSE_IsReady() != 1) {

	}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLL_MUL_8,
	LL_RCC_PREDIV_DIV_1);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while (LL_RCC_PLL_IsReady() != 1) {

	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {

	}
	LL_Init1msTick(24000000);
	LL_SetSystemCoreClock(24000000);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	TIM_InitStruct.Prescaler = 24000;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 4294967295;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM2, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM2);
	LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM2);
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	TIM_InitStruct.Prescaler = 21;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 65533;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM3, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM3);
	LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM3);
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	LL_USART_InitTypeDef USART_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/**USART2 GPIO Configuration
	 PA2   ------> USART2_TX
	 PA3   ------> USART2_RX
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART2, &USART_InitStruct);
	LL_USART_DisableIT_CTS(USART2);
	LL_USART_ConfigAsyncMode(USART2);
	LL_USART_Enable(USART2);
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	/**/
	LL_GPIO_SetOutputPin(GPIOF, LL_GPIO_PIN_1);

	/**/
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_0);

	/**/
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1);

	/**/
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);

	/**/
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

	/**/
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);

	/**/
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int preapare_msg_16bit(struct encrypt_data_t *initData, uint8_t *in,
		uint8_t *out) {
	uint8_t buff1[256];

	int ret = encodeData(initData, in, buff1, 2, msTicks);

	if (ret < 0)
		return ret;

	USART_TX_Str("Sending data: ");
	for (int i=7; i >= 0; i--) {
		USART_TX_h_VAL_without_text(buff1[i]);
	}
	USART_TX_Str("\n\r");

	encode64(buff1, out + 4);
	memcpy(out, &PREAMB, sizeof(PREAMB));


	to_diff(out, out, 20);

	return 0;
}

int preapare_msg_48bit(struct encrypt_data_t *initData, uint8_t *in,
		uint8_t *out) {
	uint8_t buff1[256] = {0};

	int ret = encodeData(initData, in, buff1, 6, msTicks);

	if (ret < 0)
		return ret;

	USART_TX_Str("Sending data: ");
	for (int i=11; i >= 0; i--) {
		USART_TX_h_VAL_without_text(buff1[i]);
	}
	USART_TX_Str("\n\r");

	encode96(buff1, out + 4);
	memcpy(out, &PREAMB, sizeof(PREAMB));


	to_diff(out, out, 28);

	return 0;
}

void preapare_msg_activate(struct encrypt_data_t *initData, uint8_t *out) {
	uint8_t buff1[256];

	encodeActivateMsg(initData, buff1, msTicks);

	USART_TX_Str("Sending data: ");
	for (int i=7; i >= 0; i--) {
		USART_TX_h_VAL_without_text(buff1[i]);
	}
	USART_TX_Str("\n\r");

	encode64(buff1, out + 4);

	memcpy(out, &PREAMB, sizeof(PREAMB));

	to_diff(out, out, 20);
}

void USART_TX(uint8_t *dt, uint16_t sz) {
	uint16_t in = 0;
	while (in < sz) {
		while (!LL_USART_IsActiveFlag_TXE(USART2)) {
		}
		LL_USART_TransmitData8(USART2, dt[in]);
		in++;
	}
}

void USART_TX_Str(char *string) {
	uint16_t size = strlen(string);
	uint16_t i = 0;
	while (i < size) {
		while (!LL_USART_IsActiveFlag_TXE(USART2)) {
		}
		LL_USART_TransmitData8(USART2, (uint8_t) string[i]);
		i++;
	}
}

void USART_TX_h_VAL(char *string, uint8_t val) {
	uint8_t s_size = strlen(string);
	uint8_t i, n = 0;
	uint8_t m_val[4];
	m_val[0] =
			(((val & 0xf0) >> 4) > 9) ?
					(((val & 0xf0) >> 4) + 0x57) : (((val & 0xf0) >> 4) + 0x30);
	m_val[1] =
			((val & 0x0f) > 9) ? ((val & 0x0f) + 0x57) : ((val & 0x0f) + 0x30);
	m_val[2] = '\n';
	m_val[3] = '\r';
	while (i < s_size) {
		while (!LL_USART_IsActiveFlag_TXE(USART2)) {
		}
		LL_USART_TransmitData8(USART2, (uint8_t) string[i]);
		i++;
	}
	while (n < 4) {
		while (!LL_USART_IsActiveFlag_TXE(USART2)) {
		}
		LL_USART_TransmitData8(USART2, m_val[n]);
		n++;
	}
}

void USART_TX_h_VAL_without_text(uint8_t val) {
	uint8_t m_val[4];
	uint8_t i, n = 0;

	m_val[0] =
			(((val & 0xf0) >> 4) > 9) ?
					(((val & 0xf0) >> 4) + 0x57) : (((val & 0xf0) >> 4) + 0x30);
	m_val[1] =
			((val & 0x0f) > 9) ? ((val & 0x0f) + 0x57) : ((val & 0x0f) + 0x30);
	while (n < 2) {
		while (!LL_USART_IsActiveFlag_TXE(USART2)) {
		}
		LL_USART_TransmitData8(USART2, m_val[n]);
		n++;
	}
}

void USART_TX_d_VAL(char *string, uint32_t val, uint8_t len) {
	uint8_t m_val[len + 2];
	for (uint8_t x = 0; x < len; x++) {
		uint32_t Os = 1;
		for (uint8_t n = 0; n < x; n++) {
			Os = Os * 10;
		}
		m_val[len - x - 1] = (val % (10 * Os)) / Os + 0x30;
	}
	m_val[len + 1] = '\n';
	m_val[len] = '\r';
	for (uint8_t i = 0; i < (strlen(string)); i++) {
		while (!LL_USART_IsActiveFlag_TXE(USART2)) {
		}
		LL_USART_TransmitData8(USART2, (uint8_t) string[i]);
	}
	for (uint8_t i = 0; i < (len + 2); i++) {
		while (!LL_USART_IsActiveFlag_TXE(USART2)) {
		}
		LL_USART_TransmitData8(USART2, (uint8_t) m_val[i]);
	}
}

uint16_t USART_RX_Str() {
	LL_USART_EnableDirectionRx(USART2);

	uint16_t ind = 0;

	//memset(u2_rx_buff, 0, sizeof(u2_rx_buff));

	while (!ind) {
		if (LL_USART_IsActiveFlag_IDLE(USART2)) {
			LL_USART_ClearFlag_IDLE(USART2);
		};

		while (!LL_USART_IsActiveFlag_IDLE(USART2)) {
			if (LL_USART_IsActiveFlag_RXNE(USART2)) {
				u2_rx_buff[ind] = (uint8_t) (USART2->RDR & 0x00FF);
				ind++;
			}
		}
	}
	LL_USART_ClearFlag_IDLE(USART2);
	LL_USART_DisableDirectionRx(USART2);

	while ((u2_rx_buff[ind - 1] == 0x0d) || (u2_rx_buff[ind - 1] == 0x0a)) {
		ind--;
	}

	return ind;
}

uint32_t AsciiToDec(uint8_t *s, uint8_t len, uint8_t number) {
	uint32_t dec_num = 0;
	uint32_t dec = 1;
	for (uint8_t i = number + len - 2; i > number - 2; i--) {
		if ((s[i] > 0x2f) && (s[i] < 0x3a)) {
			s[i] = s[i] - 0x30;
		} else {
			s[i] = 0;
			error_flag = 1;         // ERROR
		}
		dec_num = s[i] * dec + dec_num;
		dec *= 10;
	}
	return dec_num;
}

uint8_t AsciiToHex(uint8_t s_h, uint8_t s_l) {
	uint8_t hex_num;
	if ((s_h > 0x2f) && (s_h < 0x3a)) {
		s_h = s_h - 0x30;
	} else if ((s_h > 0x40) && (s_h < 0x47)) {
		s_h = s_h - 0x37;
	} else if ((s_h > 0x60) && (s_h < 0x67)) {
		s_h = s_h - 0x57;
	} else {
		s_h = 0;
		error_flag = 1;
	}
	if ((s_l > 0x2f) && (s_l < 0x3a)) {
		s_l = s_l - 0x30;
	} else if ((s_l > 0x40) && (s_l < 0x47)) {
		s_l = s_l - 0x37;
	} else if ((s_l > 0x60) && (s_l < 0x67)) {
		s_l = s_l - 0x57;
	} else {
		s_l = 0;
		error_flag = 1;
	}
	hex_num = s_l + (s_h << 4);
	return hex_num;
}
uint16_t Ascii2ToHex(uint8_t *s, uint16_t len, uint8_t number) {
	uint16_t ind = 0;
	if ((len - number) % 2) {
		for (uint16_t i = number - 1; i < len; i = i + 2) {
			if ((u2_rx_buff[i] > 0x2f) && (u2_rx_buff[i] < 0x3a)) {
				u2_rx_buff[i] = u2_rx_buff[i] - 0x30;
			}				// number
			else if ((u2_rx_buff[i] > 0x40) && (u2_rx_buff[i] < 0x47)) {
				u2_rx_buff[i] = u2_rx_buff[i] - 0x37;
			}					// C letter
			else if ((u2_rx_buff[i] > 0x60) && (u2_rx_buff[i] < 0x67)) {
				u2_rx_buff[i] = u2_rx_buff[i] - 0x57;
			} 					// m letter
			else {
				u2_rx_buff[i] = 0;
				error_flag = 1;
			}											// ERROR
			if ((u2_rx_buff[i + 1] > 0x2f) && (u2_rx_buff[i + 1] < 0x3a)) {
				u2_rx_buff[i + 1] = u2_rx_buff[i + 1] - 0x30;
			}	// number
			else if ((u2_rx_buff[i + 1] > 0x40) && (u2_rx_buff[i + 1] < 0x47)) {
				u2_rx_buff[i + 1] = u2_rx_buff[i + 1] - 0x37;
			}	    // C letter
			else if ((u2_rx_buff[i + 1] > 0x60) && (u2_rx_buff[i + 1] < 0x67)) {
				u2_rx_buff[i + 1] = u2_rx_buff[i + 1] - 0x57;
			} 	    // m letter
			else {
				u2_rx_buff[i + 1] = 0;
				error_flag = 1;
			}									// ERROR
			tx_buff[ind] = u2_rx_buff[i + 1] + (u2_rx_buff[i] << 4);
			ind++;
		}
	} else {
		error_flag = 1;
	}
	return ind;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
