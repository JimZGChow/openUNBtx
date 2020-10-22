/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <aes.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char strl[33];
uint32_t t_delay = 10;
uint32_t f_carrier;
uint32_t f_xtal = 48000000;
uint32_t FREQA, FREQB;
uint32_t TXRATE;
uint32_t BITRATE;
//uint8_t len_data = 0;
uint8_t tx_buff[256];
uint16_t PWR_dbm;
float a1;   //0<=a1<=1  txPWR
uint16_t TXPWRCOEFFB;
uint8_t rx_dt[500], tx_dt[500], tmp_str[10];
//uint8_t Ind;
uint16_t size_RX;
uint8_t error_flag = 0;
uint16_t spi_send_delay = 40000;
uint8_t shaping_filter;
uint16_t RX_STR_Size;
uint16_t RX_Data_Size;
uint8_t default_shaping_filter = 1;
uint16_t default_PWR_dbm = 11;
uint32_t default_BITRATE = 100;
uint32_t default_f_carrier = 868000000;
uint32_t transmit_timer = 0;

const uint32_t PREAMBLE = 0x97157A6F;
uint32_t ID = 0x123321;
uint32_t MIC = 0x123321;
uint8_t tm = 21;
uint32_t A = 0;
uint8_t iv[]  = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
uint8_t kp[] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
uint8_t ka[] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
uint8_t ke[16];

struct AES_ctx ctx_kp;
struct AES_ctx ctx_ke;
struct AES_ctx ctx_ka;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void USART1_TX(uint8_t *dt, uint16_t sz);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void XOR_arrays(uint8_t* a, uint8_t* b, uint8_t* c, uint32_t len) {
	for (int i=0; i<len; i++) {
		c[i] = a[i]^b[i];
	}
}

void encrypt2(struct AES_ctx* key, uint8_t* buff, uint32_t d1, uint32_t size_d1, uint32_t d2, uint32_t size_d2) {
	memset(buff, 0, 16);
	memcpy(buff, &d1, size_d1);
	memcpy(buff + size_d1, &d2, size_d2);
	AES_CBC_encrypt_buffer(key, buff, 16);
}

void encrypt1(struct AES_ctx* key, uint8_t* buff, uint32_t d1, uint32_t size_d1) {
	memset(buff, 0, 16);
	memcpy(buff, &d1, size_d1);
	AES_CBC_encrypt_buffer(key, buff, 16);
}

uint8_t encrypt_data(uint8_t* data_in, uint8_t len, uint8_t* data_out) {
	/*
	uint8_t data_encrypted[40];
	memcpy(data_encrypted, data_in, len);
	AES_CBC_encrypt_buffer(&ctx, data_encrypted, len); // DATA'
	AES_CBC_encrypt_buffer(&ctx, data_encrypted, len); // DATA''
	memcpy(data_out, &ID, 3);
	memcpy(data_out + 3, data_encrypted, len);
	memcpy(data_out + 3 + len, &MIC, 3);
	*/
	
	uint8_t datas[16];
	memset(datas, 0, 16);
	memcpy(datas, data_in, len);
	
	uint8_t gamma[16];
	encrypt2(&ctx_kp, gamma, ID, 3, tm, 1); // GAMMA
	
	XOR_arrays(datas, gamma, datas, 16); // DATA'
	
	uint32_t ne = tm/256;
	
	encrypt2(&ctx_ka, ke, ID, 3, ne, 4);
	AES_init_ctx_iv(&ctx_ke, ke, iv);    // ke
	
	uint8_t IDs_array[16];
	uint32_t IDs;
	encrypt2(&ctx_ka, IDs_array, A, 4, ne, 4);
	IDs = IDs_array[0] & (IDs_array[1] << 8) & (IDs_array[2] << 16); // ID'
	
	uint8_t gammas[16];
	encrypt2(&ctx_ke, gammas, A, 4, ne, 4); // GAMMA'
	
	uint8_t dataS[16];
	XOR_arrays(datas, gammas, dataS, 16); // DATA''
	
	uint8_t mic_array[16];
	
	return 3 + len + 3;
}

void delay(uint32_t time_delay) {
	uint32_t i;
	for (i = 0; i < time_delay; i++)
		;
}

void SPI_TR_24(uint16_t addr, uint8_t data) {
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);        // SPI SS pin
	uint8_t bit;
	uint32_t tr_data;
	uint8_t rw = 15;
	tr_data = data + (addr << 8) + (rw << 20);
	for (int16_t i = 23; i >= 0; i--) {
		bit = (tr_data >> i) & 1;
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);        //SPI CLK pin
		if (bit) {
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);          //SPI MOSI pin
		} else {
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
		}
		delay(t_delay);
		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);
		delay(t_delay);
	}
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
}

void SPI_TR_16(uint8_t addr, uint8_t data) {
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);        // SPI SS pin
	uint8_t bit;
	uint16_t tr_data;
	uint8_t rw = 1;
	tr_data = data + (addr << 8) + (rw << 15);
	for (int16_t i = 15; i >= 0; i--) {
		bit = (tr_data >> i) & 1;
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);        //SPI CLK pin
		if (bit) {
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);          //SPI MOSI pin
		} else {
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
		}
		delay(t_delay);
		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);
		delay(t_delay);
	}
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
}

void USART1_TX(uint8_t *dt, uint16_t sz) {
	uint16_t in = 0;
	while (in < sz) {
		while (!LL_USART_IsActiveFlag_TXE(USART1)) {
		}
		LL_USART_TransmitData8(USART1, dt[in]);
		in++;
	}
}
void USART_TX_Str(char *string) {
	uint16_t size = strlen(string);
	uint16_t i = 0;
	while (i < size) {
		while (!LL_USART_IsActiveFlag_TXE(USART1)) {
		}
		LL_USART_TransmitData8(USART1, (uint8_t) string[i]);
		i++;
	}
}

uint16_t USART_RX_Str() {
	LL_USART_EnableDirectionRx(USART1);

	uint16_t ind = 0;
	while (!ind) {
		if (LL_USART_IsActiveFlag_IDLE(USART1)) {
			LL_USART_ClearFlag_IDLE(USART1);
		};

		while (!LL_USART_IsActiveFlag_IDLE(USART1)) {
			if (LL_USART_IsActiveFlag_RXNE(USART1)) {
				rx_dt[ind] = (uint8_t) (USART1->RDR & 0x00FF);
				ind++;
			}
		}
	}
	LL_USART_ClearFlag_IDLE(USART1);
	LL_USART_DisableDirectionRx(USART1);
	return ind;
}

uint32_t AsciiToDec(uint8_t *s, uint8_t len, uint8_t number) {
	uint32_t dec_num = 0;
	for (int i = number - 1; i < number - 1 + len; i++) {
		if ((s[i] > 0x2f) && (s[i] < 0x3a)) {
			s[i] = s[i] - 0x30;
		} else {
			s[i] = 0;
			error_flag = 1;         // ERROR
		}
		dec_num = dec_num + (s[i] * (pow(10, (number + len - i - 2))));
	}
	return dec_num;
}

int16_t DbmToDec(uint8_t *s, uint8_t len) {
	uint16_t d_num = 0;
	for (int i = 1; i < len; i++) {
		if ((s[i] > 0x2f) && (s[i] < 0x3a)) {
			s[i] = s[i] - 0x30;
		}	// number
		else {
			s[i] = 0;
		}				      // ERROR
		d_num = d_num + (s[i] * (pow(10, (len - i - 1))));
	}
	if (s[0] == 0x2d) {
		d_num = -d_num;
	}
	return d_num;
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
	}
	if ((s_l > 0x2f) && (s_l < 0x3a)) {
		s_l = s_l - 0x30;
	} else if ((s_l > 0x40) && (s_l < 0x47)) {
		s_l = s_l - 0x37;
	} else if ((s_l > 0x60) && (s_l < 0x67)) {
		s_l = s_l - 0x57;
	} else {
		s_l = 0;
	}
	hex_num = s_l + (s_h << 4);
	return hex_num;
}
uint16_t Ascii2ToHex(uint8_t *s, uint16_t len, uint8_t number) {
	uint16_t ind = 0;
	if ((len - number) % 2) {
		for (uint16_t i = number - 1; i < len; i = i + 2) {
			if ((rx_dt[i] > 0x2f) && (rx_dt[i] < 0x3a)) {
				rx_dt[i] = rx_dt[i] - 0x30;
			}					// number
			else if ((rx_dt[i] > 0x40) && (rx_dt[i] < 0x47)) {
				rx_dt[i] = rx_dt[i] - 0x37;
			}					// C letter
			else if ((rx_dt[i] > 0x60) && (rx_dt[i] < 0x67)) {
				rx_dt[i] = rx_dt[i] - 0x57;
			} 					// m letter
			else {
				rx_dt[i] = 0;
				error_flag = 1;
			}											// ERROR
			if ((rx_dt[i + 1] > 0x2f) && (rx_dt[i + 1] < 0x3a)) {
				rx_dt[i + 1] = rx_dt[i + 1] - 0x30;
			}	// number
			else if ((rx_dt[i + 1] > 0x40) && (rx_dt[i + 1] < 0x47)) {
				rx_dt[i + 1] = rx_dt[i + 1] - 0x37;
			}	// C letter
			else if ((rx_dt[i + 1] > 0x60) && (rx_dt[i + 1] < 0x67)) {
				rx_dt[i + 1] = rx_dt[i + 1] - 0x57;
			} 	// m letter
			else {
				rx_dt[i + 1] = 0;
				error_flag = 1;
			}									// ERROR
			tx_buff[ind] = rx_dt[i + 1] + (rx_dt[i] << 4);
			ind++;
		}
	} else {
		error_flag = 1;
	}
	return ind;
}

float DbmToa1(uint16_t Dbm) {
	float a;
	switch (Dbm) {
	case (11):
		a = 0.53;
		break;
	case (10):
		a = 0.41;
		break;
	case (9):
		a = 0.345;
		break;
	case (8):
		a = 0.275;
		break;
	case (7):
		a = 0.242;
		break;
	case (6):
		a = 0.210;
		break;
	case (5):
		a = 0.175;
		break;
	case (4):
		a = 0.153;
		break;
	case (3):
		a = 0.134;
		break;
	case (2):
		a = 0.124;
		break;
	case (1):
		a = 0.110;
		break;
	case (0):
		a = 0.097;
		break;
	default:
		a = 0.53;
		break;
	}
	return a;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
//uint16_t i=0;
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
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	for (int i = 0; i < 3; i++) {                               //Indication
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);         //LED pin
		LL_mDelay(100);
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
		LL_mDelay(100);
	}
	// set RST
	SPI_TR_16(0x2, 0x80);
	delay(spi_send_delay);
	// set PWDN
	SPI_TR_16(0x2, 0x60);
	delay(spi_send_delay);
	FREQB = (((float) default_f_carrier / (float) f_xtal) * (float) pow(2, 24)
			+ (float) 0.5);
	FREQA = (uint32_t) FREQB;
	// set FREQA3
	SPI_TR_16(0x34, ((FREQA & 0xFF000000) >> 24));
	delay(spi_send_delay);
	// set FREQA2
	SPI_TR_16(0x35, ((FREQA & 0xFF0000) >> 16));
	delay(spi_send_delay);
	// set FREQA1
	SPI_TR_16(0x36, ((FREQA & 0xFF00) >> 8));
	delay(spi_send_delay);
	// set FREQA0
	SPI_TR_16(0x37, (FREQA & 0xFF));
	delay(spi_send_delay);
	// set FREQB3
	SPI_TR_16(0x3C, ((FREQA & 0xFF000000) >> 24));
	delay(spi_send_delay);
	// set FREQB2
	SPI_TR_16(0x3D, ((FREQA & 0xFF0000) >> 16));
	delay(spi_send_delay);
	// set FREQB1
	SPI_TR_16(0x3E, ((FREQA & 0xFF00) >> 8));
	delay(spi_send_delay);
	// set FREQB0
	SPI_TR_16(0x3F, (FREQA & 0xFF));
	delay(spi_send_delay);
	//set PLLRANGINGA
	SPI_TR_16(0x33, 0x18);
	delay(spi_send_delay);
	//set PLLRANGINGB
	SPI_TR_16(0x3B, 0x18);
	delay(spi_send_delay);
	//set MODULATION
	SPI_TR_16(0x10, 0x4);
	delay(spi_send_delay);
	//set ENCODING
	SPI_TR_16(0x11, 0x2);
	delay(spi_send_delay);
	//set FRAMING
	SPI_TR_16(0x12, 0x0);
	delay(spi_send_delay);
	TXRATE = (((float) default_BITRATE / (float) f_xtal) * (float) pow(2, 24)
			+ (float) 0.5);
	//set RATE
	SPI_TR_24(0x165, ((TXRATE & 0xFF0000) >> 16));
	delay(spi_send_delay);
	SPI_TR_24(0x166, ((TXRATE & 0xFF00) >> 8));
	delay(spi_send_delay);
	SPI_TR_24(0x167, (TXRATE & 0xFF));
	delay(spi_send_delay);
	//set MODCFGF filter
	SPI_TR_24(0x160, 0x3);
	delay(spi_send_delay);
	a1 = DbmToa1(PWR_dbm);
	TXPWRCOEFFB = ((float) a1 * (float) pow(2, 12) + (float) 0.5);
	//set TXPWR
	SPI_TR_24(0x168, 0x0);
	delay(spi_send_delay);
	SPI_TR_24(0x169, 0x0);
	delay(spi_send_delay);
	SPI_TR_24(0x16A, ((TXPWRCOEFFB & 0xFF00) >> 8));
	delay(spi_send_delay);
	SPI_TR_24(0x16B, (TXPWRCOEFFB & 0xFF));
	delay(spi_send_delay);
	SPI_TR_24(0x16C, 0x0);
	delay(spi_send_delay);
	SPI_TR_24(0x16D, 0x0);
	delay(spi_send_delay);
	SPI_TR_24(0x16E, 0x0);
	delay(spi_send_delay);
	SPI_TR_24(0x16F, 0x0);
	delay(spi_send_delay);
	SPI_TR_24(0x170, 0x0);
	delay(spi_send_delay);
	SPI_TR_24(0x171, 0x0);
	delay(spi_send_delay);
	// set MODCFGA
	SPI_TR_24(0x164, 0x05);  //0x5
	delay(spi_send_delay);
	// set FIFO
	SPI_TR_16(0x2, 0x67);
	delay(spi_send_delay);
	// clear error
	SPI_TR_16(0x28, 0x2);
	delay(spi_send_delay);
	// clear fifo
	SPI_TR_16(0x28, 0x3);
	delay(spi_send_delay);
	PWR_dbm = default_PWR_dbm;
	BITRATE = default_BITRATE;
	f_carrier = default_f_carrier;
	//set IRQMASK1
	SPI_TR_16(0x06, 0x0);
	delay(spi_send_delay);
	//set IRQMASK0
	SPI_TR_16(0x07, 0x40);
	delay(spi_send_delay);
	//set RADIOEVENTMASK1
	SPI_TR_16(0x08, 0x0);
	delay(spi_send_delay);
	//set RADIOEVENTMASK0
	SPI_TR_16(0x09, 0x1);
	delay(spi_send_delay);
	//set IRQINVERSION1
	SPI_TR_16(0x0A, 0x0);
	delay(spi_send_delay);
	//set IRQINVERSION0
	SPI_TR_16(0x0B, 0x40);
	delay(spi_send_delay);
	USART_TX_Str("TRANSMITTER READY\n\r");

	AES_init_ctx_iv(&ctx_kp, kp, iv);
	AES_init_ctx_iv(&ctx_ka, ka, iv);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		RX_STR_Size = USART_RX_Str();
		if ((rx_dt[0] == 'A') && (rx_dt[1] == 'T')) {
			error_flag = 0;
		}

		else if ((rx_dt[0] == 'T') && (rx_dt[1] == 'X')) {                       //////////////////////// TX
			error_flag = 0;

			// set PWDN
			SPI_TR_16(0x2, 0x60);
			delay(spi_send_delay);
			PWR_dbm = AsciiToDec(rx_dt, 2, 3);
			if ((PWR_dbm > 11) || (error_flag == 1)) {
				PWR_dbm = 11;
				error_flag = 1;
			} else {
				a1 = DbmToa1(PWR_dbm);
				TXPWRCOEFFB = ((float) a1 * (float) pow(2, 12) + (float) 0.500);
				// set PWDN
				SPI_TR_16(0x2, 0x60);
				delay(spi_send_delay);
				// set TXPWR
				SPI_TR_24(0x168, 0x0);
				delay(spi_send_delay);
				SPI_TR_24(0x169, 0x0);
				delay(spi_send_delay);
				SPI_TR_24(0x16A, ((TXPWRCOEFFB & 0xFF00) >> 8));
				delay(spi_send_delay);
				SPI_TR_24(0x16B, (TXPWRCOEFFB & 0xFF));
				delay(spi_send_delay);
				SPI_TR_24(0x16C, 0x0);
				delay(spi_send_delay);
				SPI_TR_24(0x16D, 0x0);
				delay(spi_send_delay);
				SPI_TR_24(0x16E, 0x0);
				delay(spi_send_delay);
				SPI_TR_24(0x16F, 0x0);
				delay(spi_send_delay);
				SPI_TR_24(0x170, 0x0);
				delay(spi_send_delay);
				SPI_TR_24(0x171, 0x0);
				delay(spi_send_delay);
			}
			f_carrier = AsciiToDec(rx_dt, 9, 5);
			if ((f_carrier < 864000000) || (f_carrier > 870000000)
					|| (error_flag == 1)) {
				f_carrier = 867000000;
				error_flag = 1;
			} else {
				// set PWDN
				SPI_TR_16(0x2, 0x60);
				delay(spi_send_delay);
				FREQA = (uint32_t) (((float) f_carrier / (float) f_xtal)
						* (float) pow(2, 24) + (float) 0.5);
				// set FREQA3
				SPI_TR_16(0x34, ((FREQA & 0xFF000000) >> 24));
				delay(spi_send_delay);
				// set FREQA2
				SPI_TR_16(0x35, ((FREQA & 0xFF0000) >> 16));
				delay(spi_send_delay);
				// set FREQA1
				SPI_TR_16(0x36, ((FREQA & 0xFF00) >> 8));
				delay(spi_send_delay);
				// set FREQA0
				SPI_TR_16(0x37, (FREQA & 0xFF));
				delay(spi_send_delay);
				// set FREQB3
				SPI_TR_16(0x3C, ((FREQA & 0xFF000000) >> 24));
				delay(spi_send_delay);
				// set FREQB2
				SPI_TR_16(0x3D, ((FREQA & 0xFF0000) >> 16));
				delay(spi_send_delay);
				// set FREQB1
				SPI_TR_16(0x3E, ((FREQA & 0xFF00) >> 8));
				delay(spi_send_delay);
				// set FREQB0
				SPI_TR_16(0x3F, (FREQA & 0xFF));
				delay(spi_send_delay);
			}
			RX_Data_Size = Ascii2ToHex(rx_dt, RX_STR_Size, 14);
			if ((RX_STR_Size > 14) && (RX_STR_Size < (256 + 14))
					&& (error_flag == 0)) {
				//set PLLRANGINGA
				SPI_TR_16(0x33, 0x18);
				delay(spi_send_delay);
				//set PLLRANGINGB
				SPI_TR_16(0x3B, 0x18);
				delay(spi_send_delay);
				//set MODULATION
				SPI_TR_16(0x10, 0x04);
				delay(spi_send_delay);
				//set ENCODING
				SPI_TR_16(0x11, 0xFF);
				delay(spi_send_delay);
				//set FRAMING
				SPI_TR_16(0x12, 0x0);
				delay(spi_send_delay);
				TXRATE = (((float) BITRATE / (float) f_xtal)
						* (float) pow(2, 24) + (float) 0.5);
				//set RATE
				SPI_TR_24(0x165, ((TXRATE & 0xFF0000) >> 16));
				delay(spi_send_delay);
				SPI_TR_24(0x166, ((TXRATE & 0xFF00) >> 8));
				delay(spi_send_delay);
				SPI_TR_24(0x167, (TXRATE & 0xFF));
				delay(spi_send_delay);
				//set MODCFGF filtr
				SPI_TR_24(0x160, 0x3);
				delay(spi_send_delay);
				//set FIFO
				SPI_TR_16(0x2, 0x67);
				delay(spi_send_delay);
				//clear error
				SPI_TR_16(0x28, 0x2);
				delay(spi_send_delay);
				//clear fifo
				SPI_TR_16(0x28, 0x3);
				delay(spi_send_delay);
				//set IRQMASK1
				SPI_TR_16(0x06, 0x0);
				delay(spi_send_delay);
				//set IRQMASK0
				SPI_TR_16(0x07, 0x40);
				delay(spi_send_delay);
				//set RADIOEVENTMASK1
				SPI_TR_16(0x08, 0x0);
				delay(spi_send_delay);
				//set RADIOEVENTMASK0
				SPI_TR_16(0x09, 0x1);
				delay(spi_send_delay);
				//set IRQINVERSION1
				SPI_TR_16(0x0A, 0x0);
				delay(spi_send_delay);
				//set IRQINVERSION0
				SPI_TR_16(0x0B, 0x40);
				delay(spi_send_delay);
				//set FullTX
				SPI_TR_16(0x2, 0x6D);
				delay(spi_send_delay);
				//set fifo RAW
				SPI_TR_16(0x29, 0xE1);
				delay(spi_send_delay);
				SPI_TR_16(0x29, RX_Data_Size + 1);
				delay(spi_send_delay);
				SPI_TR_16(0x29, 0x28);
				delay(spi_send_delay);
				// send fifo
				for (uint16_t u = 0; u < RX_Data_Size; u++) {
					SPI_TR_16(0x29, tx_buff[u]);
					delay(spi_send_delay);
				}
				// COMMIT
				SPI_TR_16(0x28, 0x4);
				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
				LL_mDelay(1);
				transmit_timer = 0;
				while ((LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_5))
						|| (transmit_timer > 10320)) {
					transmit_timer++;
					LL_mDelay(1);
				}
				LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
			} else {
				error_flag = 1;
			}
		}

		else if ((rx_dt[0] == 'T') && (rx_dt[1] == 'R')) {                          ///////////////////// TR
			error_flag = 0;
			RX_Data_Size = Ascii2ToHex(rx_dt, RX_STR_Size, 3);
			if ((RX_STR_Size > 3) && (RX_STR_Size < (256 + 3))
					&& (error_flag == 0)) {
				uint8_t encoded_data[40];
				//RX_Data_Size = encrypt_data(tx_buff, RX_Data_Size, encoded_data);
						
				// set PWDN
				SPI_TR_16(0x2, 0x60);
						
				//delay(spi_send_delay);
				//set PLLRANGINGA
				SPI_TR_16(0x33, 0x18);
				delay(spi_send_delay);
				//set PLLRANGINGB
				SPI_TR_16(0x3B, 0x18);
				delay(spi_send_delay);
				//set MODULATION
				SPI_TR_16(0x10, 0x04);
				delay(spi_send_delay);
				//set ENCODING
				SPI_TR_16(0x11, 0x03);
				delay(spi_send_delay);
				//set FRAMING
				SPI_TR_16(0x12, 0x0);
				delay(spi_send_delay);
				//set MODCFGF filtr
				SPI_TR_24(0x160, 0x3);
				delay(spi_send_delay);
				//set FIFO
				SPI_TR_16(0x2, 0x67);
				delay(spi_send_delay);
				//clear error
				SPI_TR_16(0x28, 0x2);
				delay(spi_send_delay);
				//clear fifo
				SPI_TR_16(0x28, 0x3);
				delay(spi_send_delay);
				//set IRQMASK1
				SPI_TR_16(0x06, 0x0);
				delay(spi_send_delay);
				//set IRQMASK0
				SPI_TR_16(0x07, 0x40);
				delay(spi_send_delay);
				//set RADIOEVENTMASK1
				SPI_TR_16(0x08, 0x0);
				delay(spi_send_delay);
				//set RADIOEVENTMASK0
				SPI_TR_16(0x09, 0x1);
				delay(spi_send_delay);
				//set IRQINVERSION1
				SPI_TR_16(0x0A, 0x0);
				delay(spi_send_delay);
				//set IRQINVERSION0
				SPI_TR_16(0x0B, 0x40);
				delay(spi_send_delay);
				//set FullTX
				SPI_TR_16(0x2, 0x6D);
				delay(spi_send_delay);
				//set fifo RAW
				SPI_TR_16(0x29, 0xE1);
				delay(spi_send_delay);
				SPI_TR_16(0x29, RX_Data_Size + 1);
				delay(spi_send_delay);
				//SPI_TR_16(0x29, 0x28);
				SPI_TR_16(0x29, 0x0FF);
				delay(spi_send_delay);
				
				// send fifo
				for (int u = 0; u < RX_Data_Size; u++) {  ////////////////////////////////////////////////////////////////
					sprintf(strl, " %.2X ", tx_buff[u]);
					USART1_TX((uint8_t*) strl, 4);
					SPI_TR_16(0x29, tx_buff[u]);
					delay(spi_send_delay);
				}
				sprintf(strl, "\r\n");
				USART1_TX((uint8_t*) strl, 2);
				// COMMIT
				SPI_TR_16(0x28, 0x4);

				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
				LL_mDelay(1);
				transmit_timer = 0;
				while ((LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_5))
						|| (transmit_timer > 10320)) {
					transmit_timer++;
					LL_mDelay(1);
				}
				LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
			} else {
				error_flag = 1;
			}
		} else if ((rx_dt[0] == 'F') && (rx_dt[1] == 'R')) {
			error_flag = 0;
			f_carrier = AsciiToDec(rx_dt, 9, 3);
			if ((f_carrier < 864000000) || (f_carrier > 870000000)
					|| (error_flag == 1)) {
				f_carrier = 867000000;
				error_flag = 1;
			} else {
				// set PWDN
				SPI_TR_16(0x2, 0x60);
				delay(spi_send_delay);
				FREQA = (uint32_t) (((float) f_carrier / (float) f_xtal)
						* (float) pow(2, 24) + (float) 0.5);
				// set FREQA3
				SPI_TR_16(0x34, ((FREQA & 0xFF000000) >> 24));
				delay(spi_send_delay);
				// set FREQA2
				SPI_TR_16(0x35, ((FREQA & 0xFF0000) >> 16));
				delay(spi_send_delay);
				// set FREQA1
				SPI_TR_16(0x36, ((FREQA & 0xFF00) >> 8));
				delay(spi_send_delay);
				// set FREQA0
				SPI_TR_16(0x37, (FREQA & 0xFF));
				delay(spi_send_delay);
				// set FREQB3
				SPI_TR_16(0x3C, ((FREQA & 0xFF000000) >> 24));
				delay(spi_send_delay);
				// set FREQB2
				SPI_TR_16(0x3D, ((FREQA & 0xFF0000) >> 16));
				delay(spi_send_delay);
				// set FREQB1
				SPI_TR_16(0x3E, ((FREQA & 0xFF00) >> 8));
				delay(spi_send_delay);
				// set FREQB0
				SPI_TR_16(0x3F, (FREQA & 0xFF));
				delay(spi_send_delay);
			}
		} else if ((rx_dt[0] == 'P') && (rx_dt[1] == 'W')) {
			error_flag = 0;
			PWR_dbm = AsciiToDec(rx_dt, 2, 3);
			if ((PWR_dbm > 11) || (error_flag == 1)) {
				PWR_dbm = 11;
				error_flag = 1;
			} else {
				a1 = DbmToa1(PWR_dbm);
				TXPWRCOEFFB = ((float) a1 * (float) pow(2, 12) + (float) 0.5);
				// set PWDN
				SPI_TR_16(0x2, 0x60);
				delay(spi_send_delay);
				// set TXPWR
				SPI_TR_24(0x168, 0x0);
				delay(spi_send_delay);
				SPI_TR_24(0x169, 0x0);
				delay(spi_send_delay);
				SPI_TR_24(0x16A, ((TXPWRCOEFFB & 0xFF00) >> 8));
				delay(spi_send_delay);
				SPI_TR_24(0x16B, (TXPWRCOEFFB & 0xFF));
				delay(spi_send_delay);
				SPI_TR_24(0x16C, 0x0);
				delay(spi_send_delay);
				SPI_TR_24(0x16D, 0x0);
				delay(spi_send_delay);
				SPI_TR_24(0x16E, 0x0);
				delay(spi_send_delay);
				SPI_TR_24(0x16F, 0x0);
				delay(spi_send_delay);
				SPI_TR_24(0x170, 0x0);
				delay(spi_send_delay);
				SPI_TR_24(0x171, 0x0);
				delay(spi_send_delay);
			}
		} else if ((rx_dt[0] == 'B') && (rx_dt[1] == 'T')) {
			error_flag = 0;
			BITRATE = AsciiToDec(rx_dt, 6, 3);
			if ((BITRATE > 125000) || (BITRATE < 100) || (error_flag == 1)) {
				BITRATE = 100;
				error_flag = 1;
			} else {
				TXRATE = (((float) BITRATE / (float) f_xtal)
						* (float) pow(2, 24) + (float) 0.5);
				// set PWDN
				SPI_TR_16(0x2, 0x60);
				delay(spi_send_delay);
				// set RATE
				SPI_TR_24(0x165, ((TXRATE & 0xFF0000) >> 16));
				delay(spi_send_delay);
				SPI_TR_24(0x166, ((TXRATE & 0xFF00) >> 8));
				delay(spi_send_delay);
				SPI_TR_24(0x167, (TXRATE & 0xFF));
				delay(spi_send_delay);
			}
		} else if ((rx_dt[0] == 'F') && (rx_dt[1] == 'L')) {
			error_flag = 0;
			shaping_filter = AsciiToDec(rx_dt, 1, 3);
			if ((shaping_filter == 0) && (error_flag == 0)) {
				// set PWDN
				SPI_TR_16(0x2, 0x60);
				// MODCFGA
				delay(spi_send_delay);
				SPI_TR_24(0x164, 0x1);
			} else if ((shaping_filter == 1) && (error_flag == 0)) {
				// set PWDN
				SPI_TR_16(0x2, 0x60);
				// set MODCFGA
				delay(spi_send_delay);
				SPI_TR_24(0x164, 0x5);
			} else {
				error_flag = 1;
				shaping_filter = 1;
			}
		} else if ((rx_dt[0] == 'S') && (rx_dt[1] == 'V')) {
			error_flag = 0;
			USART_TX_Str("Software version:1.1 \n\r");
		} else if ((rx_dt[0] == 'T') && (rx_dt[1] == 'S')) {
			error_flag = 0;
			USART_TX_Str("Transmitter settings\n\r");
			sprintf(strl, "carrier frequency:%09u\n\r", f_carrier);
			USART1_TX((uint8_t*) strl, 29);
			sprintf(strl, "transmit power:%02d\n\r", PWR_dbm);
			USART1_TX((uint8_t*) strl, 19);
			sprintf(strl, "baud rate:%06u\n\r", BITRATE);
			USART1_TX((uint8_t*) strl, 18);
			sprintf(strl, "shaping filter:%01d\n\r", shaping_filter);
			USART1_TX((uint8_t*) strl, 18);
		} else if ((rx_dt[0] == 'R') && (rx_dt[1] == 'S')) {
			error_flag = 0;
			// set RST
			SPI_TR_16(0x2, 0x80);
			delay(spi_send_delay);
			// set PWDN
			SPI_TR_16(0x2, 0x60);
			delay(spi_send_delay);
			FREQB = (((float) default_f_carrier / (float) f_xtal)
					* (float) pow(2, 24) + (float) 0.5);
			FREQA = (uint32_t) FREQB;
			// set FREQA3
			SPI_TR_16(0x34, ((FREQA & 0xFF000000) >> 24));
			delay(spi_send_delay);
			// set FREQA2
			SPI_TR_16(0x35, ((FREQA & 0xFF0000) >> 16));
			delay(spi_send_delay);
			// set FREQA1
			SPI_TR_16(0x36, ((FREQA & 0xFF00) >> 8));
			delay(spi_send_delay);
			// set FREQA0
			SPI_TR_16(0x37, (FREQA & 0xFF));
			delay(spi_send_delay);
			// set FREQB3
			SPI_TR_16(0x3C, ((FREQA & 0xFF000000) >> 24));
			delay(spi_send_delay);
			// set FREQB2
			SPI_TR_16(0x3D, ((FREQA & 0xFF0000) >> 16));
			delay(spi_send_delay);
			// set FREQB1
			SPI_TR_16(0x3E, ((FREQA & 0xFF00) >> 8));
			delay(spi_send_delay);
			// set FREQB0
			SPI_TR_16(0x3F, (FREQA & 0xFF));
			delay(spi_send_delay);
			// set PLLRANGINGA
			SPI_TR_16(0x33, 0x18);
			delay(spi_send_delay);
			// set PLLRANGINGB
			SPI_TR_16(0x3B, 0x18);
			delay(spi_send_delay);
			// set MODULATION
			SPI_TR_16(0x10, 0x4);
			delay(spi_send_delay);
			// set ENCODING
			SPI_TR_16(0x11, 0x2);
			delay(spi_send_delay);
			// set FRAMING
			SPI_TR_16(0x12, 0x0);
			delay(spi_send_delay);
			TXRATE = (((float) default_BITRATE / (float) f_xtal)
					* (float) pow(2, 24) + (float) 0.5);
			// set RATE
			SPI_TR_24(0x165, ((TXRATE & 0xFF0000) >> 16));
			delay(spi_send_delay);
			SPI_TR_24(0x166, ((TXRATE & 0xFF00) >> 8));
			delay(spi_send_delay);
			SPI_TR_24(0x167, (TXRATE & 0xFF));
			delay(spi_send_delay);
			// set MODCFGF
			SPI_TR_24(0x160, 0x3);
			delay(spi_send_delay);
			a1 = DbmToa1(PWR_dbm);
			TXPWRCOEFFB = ((float) a1 * (float) pow(2, 12) + (float) 0.5);
			//set TXPWR
			SPI_TR_24(0x168, 0x0);
			delay(spi_send_delay);
			SPI_TR_24(0x169, 0x0);
			delay(spi_send_delay);
			SPI_TR_24(0x16A, ((TXPWRCOEFFB & 0xFF00) >> 8));
			delay(spi_send_delay);
			SPI_TR_24(0x16B, (TXPWRCOEFFB & 0xFF));
			delay(spi_send_delay);
			SPI_TR_24(0x16C, 0x0);
			delay(spi_send_delay);
			SPI_TR_24(0x16D, 0x0);
			delay(spi_send_delay);
			SPI_TR_24(0x16E, 0x0);
			delay(spi_send_delay);
			SPI_TR_24(0x16F, 0x0);
			delay(spi_send_delay);
			SPI_TR_24(0x170, 0x0);
			delay(spi_send_delay);
			SPI_TR_24(0x171, 0x0);
			delay(spi_send_delay);
			// set MODCFGA
			SPI_TR_24(0x164, 0x5);
			delay(spi_send_delay);
			// set FIFO
			SPI_TR_16(0x2, 0x67);
			delay(spi_send_delay);
			// clear error
			SPI_TR_16(0x28, 0x2);
			delay(spi_send_delay);
			// clear fifo
			SPI_TR_16(0x28, 0x3);
			delay(spi_send_delay);
			//set IRQMASK1
			SPI_TR_16(0x06, 0x0);
			delay(spi_send_delay);
			//set IRQMASK0
			SPI_TR_16(0x07, 0x40);
			delay(spi_send_delay);
			//set RADIOEVENTMASK1
			SPI_TR_16(0x08, 0x0);
			delay(spi_send_delay);
			//set RADIOEVENTMASK0
			SPI_TR_16(0x09, 0x1);
			delay(spi_send_delay);
			//set IRQINVERSION1
			SPI_TR_16(0x0A, 0x0);
			delay(spi_send_delay);
			//set IRQINVERSION0
			SPI_TR_16(0x0B, 0x40);
			delay(spi_send_delay);

			PWR_dbm = default_PWR_dbm;
			BITRATE = default_BITRATE;
			f_carrier = default_f_carrier;
		} else if ((rx_dt[0] == 'S') && (rx_dt[1] == 'C')) {
			error_flag = 0;
			USART_TX_Str("Supported Commands:\n\r");
			USART_TX_Str("AT-communication test,\n\r");
			USART_TX_Str("FRxxxxxxxxx-carrier frequency,\n\r");
			USART_TX_Str("BTxxxxxx-setting the baud rate,\n\r");
			USART_TX_Str("PWxxx-transmitter power,\n\r");
			USART_TX_Str("SV-software version,\n\r");
			USART_TX_Str("TRzz...z-data transfer without parameters,\n\r");
			USART_TX_Str("TXyyxxxxxxxxxzzzz...z-data transmission with setting the transmitter power(y), carrier frequency(x) and data (z),\n\r");
			USART_TX_Str("TS-transmitter settings,\n\r");
			USART_TX_Str("RS-module reboot,\n\r");
			USART_TX_Str("SC-command list.\n\r");
		} else {
			error_flag = 1;
		}
		if (!error_flag) {
			USART_TX_Str("OK\n\r");
		} else {
			USART_TX_Str("ERROR\n\r");
		}
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
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_8);
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
	LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	LL_USART_InitTypeDef USART_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/**USART1 GPIO Configuration
	 PA2   ------> USART1_TX
	 PA3   ------> USART1_RX
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

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART1, &USART_InitStruct);
	LL_USART_DisableIT_CTS(USART1);
	LL_USART_ConfigAsyncMode(USART1);
	LL_USART_Enable(USART1);
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

	/**/
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);

	/**/
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);

	/**/
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);

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

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
