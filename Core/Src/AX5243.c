/*
 * AD5243.c
 *
 *  Created on: 29.04.2021
 *      Author: S.K.TUSUR
 */
#include "AX5243.h"

uint32_t f_xtal = 48000000;
uint32_t FREQA;
uint16_t TXPWRCOEFFB;
uint8_t  def_shaping_filter = default_shaping;
uint16_t def_PWR_dbm = default_PWR_dbm;
uint32_t def_f_carrier = default_f_carrier;
uint32_t f_carrier_calibration = 1638;



void AX5243_u_delay(uint16_t delay){
	((AX5243_used_TIM->CNT) = (0));
	while((AX5243_used_TIM->CNT) < (delay));
}



uint32_t DbmToa1(uint8_t Dbm){
uint32_t a;
uint32_t a_table[] = {97, 110, 124, 134, 153, 175, 210, 242, 275, 345, 410, 530};
if(Dbm < 12){
a = a_table[Dbm];
}
else{
a = a_table[11];
}
return a;
}

void SPI_TR_24(uint16_t addr, uint8_t data)
{

	  AX5243_SS_PORT->BRR = (1<<AX5243_SS_PIN);
	  uint8_t bit;
	  uint32_t tr_data;
	  uint8_t rw = 15;
	  tr_data = data + (addr<<8) + (rw<<20);
    for(int16_t i = 23; i >= 0; i--)
	{
		bit = (tr_data >>i) & 1;
		AX5243_CLK_PORT->BRR = (1<<AX5243_CLK_PIN);
		if(bit)
		{
	    AX5243_MOSI_PORT->BSRR = (1<<AX5243_MOSI_PIN);
		}
		else
		{
		AX5243_MOSI_PORT->BRR = (1<<AX5243_MOSI_PIN);
		}
		AX5243_u_delay(AX5243_delay);
		AX5243_CLK_PORT->BSRR = (1<<AX5243_CLK_PIN);
		AX5243_u_delay(AX5243_delay);
	}
      AX5243_CLK_PORT->BRR = (1<<AX5243_CLK_PIN);
      AX5243_SS_PORT->BSRR = (1<<AX5243_SS_PIN);
}

void SPI_TR_16(uint8_t addr, uint8_t data)
{
	  AX5243_SS_PORT->BRR = (1<<AX5243_SS_PIN);
	  uint8_t bit;
	  uint16_t tr_data;
	  uint8_t rw = 1;
	  tr_data = data + (addr<<8) + (rw<<15);
    for(int16_t i = 15; i >= 0; i--)
	{
		bit = (tr_data >>i) & 1;
		AX5243_CLK_PORT->BRR = (1<<AX5243_CLK_PIN);
		if(bit)
		{
		AX5243_MOSI_PORT->BSRR = (1<<AX5243_MOSI_PIN);
		}
		else
		{
		AX5243_MOSI_PORT->BRR = (1<<AX5243_MOSI_PIN);
		}
		AX5243_u_delay(AX5243_delay);
		AX5243_CLK_PORT->BSRR = (1<<AX5243_CLK_PIN);
		AX5243_u_delay(AX5243_delay);
	}
      AX5243_CLK_PORT->BRR = (1<<AX5243_CLK_PIN);
      AX5243_SS_PORT->BSRR = (1<<AX5243_SS_PIN);

}



void AX5243_init() {

	SPI_TR_16(0x2, 0x60); // set PWDN
	AX5243_u_delay(10000);
	SPI_TR_16(0x30, 0x09); //PLLLOOP
	SPI_TR_16(0x32, 0x01); //PLLVCODIV
	SPI_TR_24(0xF10, 0x04);
	SPI_TR_24(0xF35, 0x11);
	SPI_TR_24(0xF34, 0x08); //PERFTUNE52
	SPI_TR_16(0x21, 0x08); //PINFUNCSYSCLK
	FREQA = (uint32_t)((((uint64_t)def_f_carrier + (uint64_t)f_carrier_calibration)*(uint64_t)1000000000/(uint64_t)f_xtal)*(uint64_t)16777217/(uint64_t)1000000000);
	SPI_TR_16(0x34, ((FREQA & 0xFF000000) >> 24)); // set FREQA3
	SPI_TR_16(0x35, ((FREQA & 0xFF0000) >> 16)); // set FREQA2
	SPI_TR_16(0x36, ((FREQA & 0xFF00) >> 8)); 	// set FREQA1
	SPI_TR_16(0x37, (FREQA & 0xFF)); 	// set FREQA0
	SPI_TR_16(0x3C, ((FREQA & 0xFF000000) >> 24)); 	// set FREQB3
	SPI_TR_16(0x3D, ((FREQA & 0xFF0000) >> 16)); // set FREQB2
	SPI_TR_16(0x3E, ((FREQA & 0xFF00) >> 8)); 	// set FREQB1
	SPI_TR_16(0x3F, (FREQA & 0xFF));	// set FREQB0
	SPI_TR_16(0x2, 0x65);	// set Standby
	AX5243_u_delay(32000);
	AX5243_u_delay(32000);//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	SPI_TR_16(0x33, 0x18);	//set PLLRANGINGA
	AX5243_u_delay(32000);
	AX5243_u_delay(32000);//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	SPI_TR_24(0x164, 0x05);	//MODCFGA
	SPI_TR_24(0xf5f, 0xe9);	//MODCFGP
	TXPWRCOEFFB = ((DbmToa1(def_PWR_dbm)*4096)+500)/1000;
	SPI_TR_24(0x168, 0x0);//set TXPWR
	SPI_TR_24(0x169, 0x0);
	SPI_TR_24(0x16A, ((TXPWRCOEFFB & 0xFF00) >> 8));
	SPI_TR_24(0x16B, (TXPWRCOEFFB & 0xFF));
	SPI_TR_24(0x16C, 0x0);
	SPI_TR_24(0x16D, 0x0);
	SPI_TR_24(0x16E, 0x0);
	SPI_TR_24(0x16F, 0x0);
	SPI_TR_24(0x170, 0x0);
	SPI_TR_24(0x171, 0x0);
	SPI_TR_16(0x10, 0x4);	//set MODULATION
	SPI_TR_16(0x11, 0x0);	//set ENCODING
	SPI_TR_16(0x12, 0x0);	//set FRAMING
	SPI_TR_24(0x165, ((0x23 & 0xFF0000) >> 16));	//set RATE
	SPI_TR_24(0x166, ((0x23 & 0xFF00) >> 8));
	SPI_TR_24(0x167, (0x23 & 0xFF));
	SPI_TR_16(0x2, 0x67);	//set FIFO
	SPI_TR_16(0x28, 0x2);	//clear error
	SPI_TR_16(0x28, 0x3);	//clear FIFO
	SPI_TR_16(0x06, 0x0);	//set IRQMASK1
	SPI_TR_16(0x07, 0x40);	//set IRQMASK0
	SPI_TR_16(0x08, 0x0);	//set RADIOEVENTMASK1
	SPI_TR_16(0x09, 0x1);	//set RADIOEVENTMASK0
	SPI_TR_16(0x0A, 0x0);	//set IRQINVERSION1
	SPI_TR_16(0x0B, 0x40);	//set IRQINVERSION0
}

uint8_t AX5243_transmit(uint8_t* data, uint16_t RX_Data_Size) {
	uint32_t transmit_timer = 0;
	uint8_t error_flag = 0;

	if((RX_Data_Size>0)&&(RX_Data_Size<(256+0))){
	SPI_TR_16(0x2, 0x60);// set PWDN
	AX5243_u_delay(10000);
	SPI_TR_16(0x30, 0x09);  //PLLLOOP
	SPI_TR_16(0x32, 0x01);   //PLLVCODIV
	SPI_TR_24(0xF10, 0x04);
	SPI_TR_24(0xF35, 0x11);
	SPI_TR_24(0xF34, 0x08); //PERFTUNE52
	SPI_TR_16(0x21, 0x08); //PINFUNCSYSCLK
	SPI_TR_16(0x2, 0x65);// set Standby
	AX5243_u_delay(32000);
	AX5243_u_delay(32000);//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	SPI_TR_16(0x33, 0x18);//set PLLRANGINGA
	AX5243_u_delay(32000);
	AX5243_u_delay(32000);
	AX5243_u_delay(32000);
	AX5243_u_delay(32000);//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	SPI_TR_16(0x10, 0x4);//set MODULATION
	SPI_TR_16(0x11, 0x0);//set ENCODING
	SPI_TR_16(0x12, 0x0);//set FRAMING


	SPI_TR_16(0x2, 0x67);//set FIFO
	SPI_TR_16(0x28, 0x2); //clear error
	SPI_TR_16(0x28, 0x3);	//clear FIFO
	SPI_TR_16(0x06, 0x0);		//set IRQMASK1
	SPI_TR_16(0x07, 0x40);		//set IRQMASK0
	SPI_TR_16(0x08, 0x0);//set RADIOEVENTMASK1
	SPI_TR_16(0x09, 0x1);//set RADIOEVENTMASK0
	SPI_TR_16(0x0A, 0x0); //set IRQINVERSION1
	SPI_TR_16(0x0B, 0x40); 		//set IRQINVERSION0
	SPI_TR_16(0x2, 0x6D);//set FullTX
	SPI_TR_16(0x29, 0xE1); //set FIFO RAW
	SPI_TR_16(0x29, RX_Data_Size+1);
	SPI_TR_16(0x29, 0x28);
		// send FIFO
		for(int u = 0; u<RX_Data_Size; u++)
		{
			SPI_TR_16(0x29, data[u]);
		}
		SPI_TR_16(0x28, 0x4);// COMMIT
		//LL_GPIO_SetOutputPin(LED_PIN);
		LED_PORT->BSRR = (1<<LED_PIN);
		AX5243_u_delay(1070);
		transmit_timer = 0;
		while((AX5243_IRQ_PORT->IDR & (1<<AX5243_IRQ_PIN)) ||(transmit_timer>10320)){
		//while(( LL_GPIO_IsInputPinSet(AX5243_IRQ)) ||(transmit_timer>10320)){
		transmit_timer++;
		AX5243_u_delay(1070);

		}
		//LL_GPIO_ResetOutputPin(LED_PIN);
		LED_PORT->BRR = (1<<LED_PIN);
	}
	else{
	error_flag=1;

	}
	return error_flag;
}

/*uint8_t AX5243_st_transmit(uint8_t* data, uint16_t RX_Data_Size, uint32_t f_carrier, uint16_t PWR_dbm) {
	uint8_t error_flag = 0;
	uint32_t transmit_timer = 0;
	SPI_TR_16(0x2, 0x60);// set PWDN
	AX5243_u_delay(10000);
	SPI_TR_16(0x30, 0x09);  //PLLLOOP
	SPI_TR_16(0x32, 0x01);   	//PLLVCODIV
	SPI_TR_24(0xF10, 0x04);
	SPI_TR_24(0xF35, 0x11);
	SPI_TR_24(0xF34, 0x08); //PERFTUNE52
	SPI_TR_16(0x21, 0x08); //PINFUNCSYSCLK
		if((f_carrier<864000000) || (f_carrier>870000000)){
			f_carrier = 867000000;
			error_flag = 1;
		}
		else{
			FREQA = ((f_carrier+f_carrier_calibration)/f_xtal)*16777216;
			SPI_TR_16(0x34, ((FREQA & 0xFF000000) >> 24));	// set FREQA3
			SPI_TR_16(0x35, ((FREQA & 0xFF0000) >> 16));	// set FREQA2
			SPI_TR_16(0x36, ((FREQA & 0xFF00) >> 8));// set FREQA1
			SPI_TR_16(0x37, (FREQA & 0xFF));	// set FREQA0
			SPI_TR_16(0x3C, ((FREQA & 0xFF000000) >> 24));// set FREQB3
			SPI_TR_16(0x3D, ((FREQA & 0xFF0000) >> 16));// set FREQB2
			SPI_TR_16(0x3E, ((FREQA & 0xFF00) >> 8));// set FREQB1
			SPI_TR_16(0x3F, (FREQA & 0xFF));// set FREQB0
		}
			SPI_TR_16(0x2, 0x65);// set Standby
			AX5243_u_delay(32000);
			AX5243_u_delay(32000);//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			SPI_TR_16(0x33, 0x18);//set PLLRANGINGA
			AX5243_u_delay(32000);
			AX5243_u_delay(32000);//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		if(PWR_dbm>11){
			PWR_dbm = 11;
			error_flag = 1;
		}
		else{
			TXPWRCOEFFB = ((DbmToa1(PWR_dbm)*4096)+500)/1000;
			SPI_TR_16(0x2, 0x60);   // set PWDN
			SPI_TR_24(0x168, 0x0);	// set TXPWR
			SPI_TR_24(0x169, 0x0);
			SPI_TR_24(0x16A, ((TXPWRCOEFFB & 0xFF00) >> 8));
			SPI_TR_24(0x16B, (TXPWRCOEFFB & 0xFF));
			SPI_TR_24(0x16C, 0x0);
			SPI_TR_24(0x16D, 0x0);
			SPI_TR_24(0x16E, 0x0);
			SPI_TR_24(0x16F, 0x0);
			SPI_TR_24(0x170, 0x0);
			SPI_TR_24(0x171, 0x0);
		}

		if((RX_Data_Size>0)&&(RX_Data_Size<(256+0))&&(error_flag ==0)){
			SPI_TR_16(0x10, 0x4);//set MODULATION
			SPI_TR_16(0x11, 0x0);	//set ENCODING
			SPI_TR_16(0x12, 0x0);	//set FRAMING
			SPI_TR_24(0x165, ((0x23 & 0xFF0000) >> 16));//set RATE
			SPI_TR_24(0x166, ((0x23 & 0xFF00) >> 8));
			SPI_TR_24(0x167, (0x23 & 0xFF));
			SPI_TR_16(0x2, 0x67);//set FIFO
			SPI_TR_16(0x28, 0x2);	//clear error
			SPI_TR_16(0x28, 0x3);//clear FIFO
			SPI_TR_16(0x06, 0x0);//set IRQMASK1
			SPI_TR_16(0x07, 0x40);//set IRQMASK0
			SPI_TR_16(0x08, 0x0);//set RADIOEVENTMASK1
			SPI_TR_16(0x09, 0x1);//set RADIOEVENTMASK0
			SPI_TR_16(0x0A, 0x0);//set IRQINVERSION1
			SPI_TR_16(0x0B, 0x40);	//set IRQINVERSION0
			SPI_TR_16(0x2, 0x6D);//set FullTX
			SPI_TR_16(0x29, 0xE1);//set FIFO RAW
			SPI_TR_16(0x29, RX_Data_Size+1);
			SPI_TR_16(0x29, 0x28);
			// send FIFO
			for(uint16_t u = 0; u<RX_Data_Size; u++)
			{
				SPI_TR_16(0x29, data[u]);
			}
			SPI_TR_16(0x28, 0x4);	// COMMIT
			LED_PORT->BSRR = (1<<LED_PIN);
			//LL_GPIO_SetOutputPin(LED_PIN);
			AX5243_u_delay(1070);
			transmit_timer = 0;
			while((AX5243_IRQ_PORT->IDR & (1<<AX5243_IRQ_PIN)) ||(transmit_timer>10320)){
			//while(( LL_GPIO_IsInputPinSet(AX5243_IRQ)) ||(transmit_timer>10320)){
			transmit_timer++;
			AX5243_u_delay(1070);
			}
			LED_PORT->BRR = (1<<LED_PIN);
			//LL_GPIO_ResetOutputPin(LED_PIN);
		}
		else{
		error_flag=1;
		}
	return error_flag;
}
*/

uint8_t AX5243_set_fr(uint32_t f_carrier) {
	uint8_t error_flag = 0;
		if((f_carrier<864000000) || (f_carrier>870000000)){
			f_carrier = 867000000;
			error_flag = 1;
		}
		else{
			SPI_TR_16(0x2, 0x60);// set PWDN
			AX5243_u_delay(10000);
			FREQA = (uint32_t)((((uint64_t)f_carrier + (uint64_t)f_carrier_calibration)*(uint64_t)1000000000/(uint64_t)f_xtal)*(uint64_t)16777217/(uint64_t)1000000000);
			SPI_TR_16(0x34, ((FREQA & 0xFF000000) >> 24));// set FREQA3
			SPI_TR_16(0x35, ((FREQA & 0xFF0000) >> 16));// set FREQA2
			SPI_TR_16(0x36, ((FREQA & 0xFF00) >> 8));// set FREQA1
			SPI_TR_16(0x37, (FREQA & 0xFF));// set FREQA0
			SPI_TR_16(0x3C, ((FREQA & 0xFF000000) >> 24));// set FREQB3
			SPI_TR_16(0x3D, ((FREQA & 0xFF0000) >> 16));// set FREQB2
			SPI_TR_16(0x3E, ((FREQA & 0xFF00) >> 8));// set FREQB1
			SPI_TR_16(0x3F, (FREQA & 0xFF));// set FREQB0
		}
		return error_flag;
}



uint8_t AX5243_set_pw(uint16_t PWR_dbm) {
	uint8_t error_flag = 0;
		if(PWR_dbm>11){
			PWR_dbm = 11;
			error_flag = 1;
		}
		else{
			TXPWRCOEFFB = ((DbmToa1(PWR_dbm)*4096)+500)/1000;
			SPI_TR_16(0x2, 0x60);// set PWDN
			SPI_TR_24(0x168, 0x0);// set TXPWR
			SPI_TR_24(0x169, 0x0);
			SPI_TR_24(0x16A, ((TXPWRCOEFFB & 0xFF00) >> 8));
			SPI_TR_24(0x16B, (TXPWRCOEFFB & 0xFF));
			SPI_TR_24(0x16C, 0x0);
			SPI_TR_24(0x16D, 0x0);
			SPI_TR_24(0x16E, 0x0);
			SPI_TR_24(0x16F, 0x0);
			SPI_TR_24(0x170, 0x0);
			SPI_TR_24(0x171, 0x0);

		}
		return error_flag;
}

uint8_t AX5243_set_fl(uint8_t shaping_filter) {
	uint8_t error_flag = 0;
		if(shaping_filter==0){
			SPI_TR_16(0x2, 0x60);// set PWDN
			SPI_TR_24(0x164, 0x1);// MODCFGA
		}
		else if(shaping_filter==1){
			SPI_TR_16(0x2, 0x60);// set PWDN
			SPI_TR_24(0x164, 0x5);	// set MODCFGA
		}
		else{
		error_flag = 1;
		shaping_filter=1;
		}
		return error_flag;
}

uint8_t AX5243_set_pd(uint8_t p_d) {
	uint8_t error_flag = 0;
		if(p_d==0){
			SPI_TR_16(0x2, 0x60);       // set PWDN
			SPI_TR_24(0xf5f, 0xe1);		//dMODCFGP
		}
		else if(p_d==1){
			SPI_TR_16(0x2, 0x60);// set PWDN
			SPI_TR_24(0xf5f, 0xe9);	//dMODCFGP
		}
		else if(p_d==2){
			SPI_TR_16(0x2, 0x60);// set PWDN
			SPI_TR_24(0xf5f, 0xf1);	//dMODCFGP
		}
		else{
		error_flag = 1;
		}
		return error_flag;
}



void AX5243_rs() {
	SPI_TR_16(0x2, 0x60);	// set PWDN
	AX5243_u_delay(10000);
	SPI_TR_16(0x30, 0x09);  	//PLLLOOP
	SPI_TR_16(0x32, 0x01);   		//PLLVCODIV
	SPI_TR_24(0xF10, 0x04);
	SPI_TR_24(0xF35, 0x11);
	SPI_TR_24(0xF34, 0x08); //PERFTUNE52
	SPI_TR_16(0x21, 0x08); //PINFUNCSYSCLK
	FREQA = (uint32_t)((((uint64_t)def_f_carrier + (uint64_t)f_carrier_calibration)*(uint64_t)1000000000/(uint64_t)f_xtal)*(uint64_t)16777217/(uint64_t)1000000000);
	SPI_TR_16(0x34, ((FREQA & 0xFF000000) >> 24));// set FREQA3
	SPI_TR_16(0x35, ((FREQA & 0xFF0000) >> 16));// set FREQA2
	SPI_TR_16(0x36, ((FREQA & 0xFF00) >> 8));	// set FREQA1
	SPI_TR_16(0x37, (FREQA & 0xFF));	// set FREQA0
	SPI_TR_16(0x3C, ((FREQA & 0xFF000000) >> 24));	// set FREQB3
	SPI_TR_16(0x3D, ((FREQA & 0xFF0000) >> 16));// set FREQB2
	SPI_TR_16(0x3E, ((FREQA & 0xFF00) >> 8));	// set FREQB1
	SPI_TR_16(0x3F, (FREQA & 0xFF));	// set FREQB0
	SPI_TR_16(0x2, 0x65);			// set Standby
	AX5243_u_delay(32000);
	AX5243_u_delay(32000);//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	SPI_TR_16(0x33, 0x18);			//set PLLRANGINGA
	AX5243_u_delay(32000);
	AX5243_u_delay(32000);//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    SPI_TR_24(0x164, 0x05);		//MODCFGA
	SPI_TR_24(0xf5f, 0xe9);			//MODCFGP
	//set MODCFGF filter
	//SPI_TR_24(0x160, 0x3);
	TXPWRCOEFFB = ((DbmToa1(def_PWR_dbm)*4096)+500)/1000;
	SPI_TR_24(0x168, 0x0); //set TXPWR
	SPI_TR_24(0x169, 0x0);
	SPI_TR_24(0x16A, ((TXPWRCOEFFB & 0xFF00) >> 8));
	SPI_TR_24(0x16B, (TXPWRCOEFFB & 0xFF));
	SPI_TR_24(0x16C, 0x0);
	SPI_TR_24(0x16D, 0x0);
	SPI_TR_24(0x16E, 0x0);
	SPI_TR_24(0x16F, 0x0);
	SPI_TR_24(0x170, 0x0);
	SPI_TR_24(0x171, 0x0);
	SPI_TR_16(0x10, 0x4);	//set MODULATION
	SPI_TR_16(0x11, 0x0);	//set ENCODING
	SPI_TR_16(0x12, 0x0);	//set FRAMING
	SPI_TR_24(0x165, ((0x23 & 0xFF0000) >> 16));	//set RATE
	SPI_TR_24(0x166, ((0x23 & 0xFF00) >> 8));
	SPI_TR_24(0x167, (0x23 & 0xFF));
	SPI_TR_16(0x2, 0x67);	//set FIFO
	SPI_TR_16(0x28, 0x2);	//clear error
	SPI_TR_16(0x28, 0x3);	//clear FIFO
	SPI_TR_16(0x06, 0x0);	//set IRQMASK1
	SPI_TR_16(0x07, 0x40);	//set IRQMASK0
	SPI_TR_16(0x08, 0x0);	//set RADIOEVENTMASK1
	SPI_TR_16(0x09, 0x1);	//set RADIOEVENTMASK0
	SPI_TR_16(0x0A, 0x0);	//set IRQINVERSION1
	SPI_TR_16(0x0B, 0x40);	//set IRQINVERSION0
}

