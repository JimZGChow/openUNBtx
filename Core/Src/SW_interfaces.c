/*
 * SW_interfaces.c
 *
 *  Created on: 29.04.2021
 *      Author: S.K.TUSUR
 */
#include "SW_interfaces.h"

uint8_t id;



void cdelay(uint16_t ms_delay){
((used_TIM->CNT) = (0));
while((used_TIM->CNT) < (ms_delay));
}

void I2C_Start(void)
{
	uint8_t ind = 0;
        I2C_SDA_PORT->BSRR = (1<<I2C_SDA_PIN);
        cdelay(ti_delay);
        I2C_SCL_PORT->BSRR = (1<<I2C_SCL_PIN);
        cdelay(ti_delay);

        while((!(I2C_SDA_PORT->IDR & (1<<I2C_SDA_PIN))) && (ind < 20))
        {
        I2C_SCL_PORT->BRR  = (1<<I2C_SCL_PIN);
        cdelay(ti_delay);
        I2C_SCL_PORT->BSRR = (1<<I2C_SCL_PIN);
        cdelay(ti_delay);
        ind++;
        }
        I2C_SDA_PORT->BRR = (1<<I2C_SDA_PIN);
        cdelay(ti_delay);
        I2C_SCL_PORT->BRR = (1<<I2C_SCL_PIN);
        cdelay(ti_delay);
}
void I2C_Stop(void)
{
        I2C_SDA_PORT->BRR = (1<<I2C_SDA_PIN);
        cdelay(ti_delay);
        I2C_SCL_PORT->BSRR = (1<<I2C_SCL_PIN);
        cdelay(ti_delay);
        I2C_SDA_PORT->BSRR = (1<<I2C_SDA_PIN);
        cdelay(ti_delay);
}

uint8_t I2C_Write_Byte(uint8_t data)
{
       uint8_t i;
       uint8_t ACK;
       for(i=0;i<8;i++)
       {
       if(data & 0x80)
       {
       I2C_SDA_PORT->BSRR = (1<<I2C_SDA_PIN);
       }
       else
       {
       I2C_SDA_PORT->BRR = (1<<I2C_SDA_PIN);
       }
       cdelay(ti_delay);
       I2C_SCL_PORT->BSRR = (1<<I2C_SCL_PIN);
       cdelay(ti_delay);
       I2C_SCL_PORT->BRR = (1<<I2C_SCL_PIN);
       data=data<<1;
       }
       cdelay(ti_delay);
       I2C_SCL_PORT->BSRR = (1<<I2C_SCL_PIN);
       cdelay(ti_delay);
       ACK = !(I2C_SDA_PORT->IDR & (1<<I2C_SDA_PIN));
       I2C_SCL_PORT->BRR = (1<<I2C_SCL_PIN);
       I2C_SDA_PORT->BRR = (1<<I2C_SDA_PIN);
       return ACK;
}

uint8_t I2C_Read_Byte(uint8_t ACK)
{
        uint8_t i;
        uint8_t data = 0;

        I2C_SDA_PORT->BSRR = (1<<I2C_SDA_PIN);
      for(i=0;i<8;i++)
        {
    	cdelay(ti_delay);
        I2C_SCL_PORT->BSRR = (1<<I2C_SCL_PIN);
        cdelay(ti_delay);
        data<<=1;
      if(I2C_SDA_PORT->IDR & (1<<I2C_SDA_PIN))
        data++;
        I2C_SCL_PORT->BRR = (1<<I2C_SCL_PIN);
        }
      if (ACK)
        I2C_SDA_PORT->BRR = (1<<I2C_SDA_PIN);
        cdelay(ti_delay);
        I2C_SCL_PORT->BSRR = (1<<I2C_SCL_PIN);
        cdelay(ti_delay);
        I2C_SCL_PORT->BRR = (1<<I2C_SCL_PIN);
        I2C_SDA_PORT->BSRR = (1<<I2C_SDA_PIN);
        return data;
}


void SPI2_Write_Byte(uint8_t addr, uint8_t data)
{
	SPI_SENS_CLK_PORT->BSRR = (1<<SPI_SENS_CLK_PIN);
	SPI_SENS_SS_PORT->BRR = (1<<SPI_SENS_SS_PIN);
	  uint8_t bit;
	  uint16_t tr_data = 0;
	  uint8_t rw = 0;
	  tr_data = data + (((addr)&0x7F)<<8) + (rw<<15);
    for(uint8_t i = 16; i > 0; i--)
	{
		bit = (tr_data >>(i-1)) & 1;
		SPI_SENS_CLK_PORT->BRR = (1<<SPI_SENS_CLK_PIN);
		if(bit)
		{
		SPI_SENS_MOSI_PORT->BSRR = (1<<SPI_SENS_MOSI_PIN);
		}
		else
		{
		SPI_SENS_MOSI_PORT->BRR = (1<<SPI_SENS_MOSI_PIN);
		}
		cdelay(ts_delay);
		SPI_SENS_CLK_PORT->BSRR = (1<<SPI_SENS_CLK_PIN);
		cdelay(ts_delay);
	}
    SPI_SENS_SS_PORT->BSRR = (1<<SPI_SENS_SS_PIN);
    SPI_SENS_CLK_PORT->BSRR = (1<<SPI_SENS_CLK_PIN);
}

uint8_t SPI2_Read_Byte(uint8_t addr)
{
	SPI_SENS_CLK_PORT->BSRR = (1<<SPI_SENS_CLK_PIN);
	SPI_SENS_SS_PORT->BRR = (1<<SPI_SENS_SS_PIN);
	  uint8_t bit;
	  uint16_t tr_data = 0;
	  uint8_t rs_data = 0;
	  uint8_t rw = 1;
	  tr_data = (((addr)&0x7F)<<8) + (rw<<15);
    for(uint8_t i = 16; i > 8; i--)
	{
		bit = (tr_data >>(i-1)) & 1;
		SPI_SENS_CLK_PORT->BRR = (1<<SPI_SENS_CLK_PIN);
		if(bit)
		{
		SPI_SENS_MOSI_PORT->BSRR = (1<<SPI_SENS_MOSI_PIN);
		}
		else
		{
	  SPI_SENS_MOSI_PORT->BRR = (1<<SPI_SENS_MOSI_PIN);
		}
		cdelay(ts_delay);
		SPI_SENS_CLK_PORT->BSRR = (1<<SPI_SENS_CLK_PIN);
		cdelay(ts_delay);
	}

    for(uint8_t i = 8; i > 0; i--)
	{
		SPI_SENS_CLK_PORT->BRR = (1<<SPI_SENS_CLK_PIN);
		cdelay(ts_delay);
		SPI_SENS_CLK_PORT->BSRR = (1<<SPI_SENS_CLK_PIN);
	    if(SPI_SENS_MISO_PORT->IDR & (1<<SPI_SENS_MISO_PIN)){
	    rs_data = rs_data | (0x1)<<(i-1);
	    }
		cdelay(ts_delay);
	}
    SPI_SENS_CLK_PORT->BSRR = (1<<SPI_SENS_CLK_PIN);
    SPI_SENS_SS_PORT->BSRR = (1<<SPI_SENS_SS_PIN);
	  return rs_data;
}

