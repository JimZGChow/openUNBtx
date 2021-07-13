#ifndef OPENUNBDECRYPTERLL_H
#define OPENUNBDECRYPTERLL_H

#include <stdint.h>
#include <string.h>

#include "encrypt/aes.h"
#include "encrypt/OpenUNBTypes.h"

#define MS2S(x)		(x/1000)    // ms to sec
#define S2M(x)		(x/60)		// sec to min
#define M2H(x)		(x/60)		// min to hours
#define M2S(x)		(x*60)
#define MS2M(x)		(S2M(MS2S(x)))		// ms to min
#define MS2H(x)		(M2H(S2M(MS2S(x))))	// ms to hours

#define H2S(x)		(x*60*60)	// hours to sec

#define EPOCH_DURATION_TIME	(M2S(4))

void initEncrypter(struct encryptData* enc_data);
void encodeActivateMsg(struct encryptData* enc_data, uint8_t* out, time_t time);
void encode16Bit(struct encryptData* enc_data, uint8_t* in, uint8_t* out, time_t time);
void encode48Bit(struct encryptData* enc_data, uint8_t* in, uint8_t* out, time_t time);

uint128_256_t getKa(uint128_256_t K0, uint16_t Na);

uint24a_t getDevAddr(uint128_256_t Ka, uint24a_t Ne);

uint128_256_t getKm(uint128_256_t Ka, uint24a_t Ne);

uint128_256_t getKe(uint128_256_t Ka, uint24a_t Ne);

uint16_t cryptoMacPayload16(uint16_t macPayload, uint128_256_t Ke, uint16_t Nn);
uint48a_t cryptoMacPayload48(uint48a_t macPayload, uint128_256_t Ke, uint16_t Nn);

uint24a_t getMIC16(uint128_256_t Km, uint24a_t DevAddr, uint16_t cryptoMacPayload, uint16_t Nn);
uint24a_t getMIC48(uint128_256_t Km, uint24a_t DevAddr, uint48a_t cryptoMacPayload, uint16_t Nn);

#endif
