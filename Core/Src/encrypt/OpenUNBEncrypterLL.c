#include "encrypt/OpenUNBEncrypterLL.h"


void initEncrypter(struct encryptData* enc_data) {
	enc_data->Ka = getKa(enc_data->K0, enc_data->Na);
}

void encodeActivateMsg(struct encryptData* enc_data, uint8_t* out, time_t time) {
	enc_data->initTime = MS2S(time); // to sec
	enc_data->Ne.ud = 0;
	enc_data->Ke = getKe(enc_data->Ka, enc_data->Ne);
	enc_data->Km = getKm(enc_data->Ka, enc_data->Ne);

	enc_data->DevAddr = getDevAddr(enc_data->Ka, enc_data->Ne);

	uint24a_t devAddr24 = {0};
	uint16_t payload = enc_data->Na;
	uint24a_t MIC = {0};

	devAddr24.ud = crc24(enc_data->DevID, enc_data->DevID_len);
	MIC = getMIC16(enc_data->Km, devAddr24, payload, 0);

	memcpy(out, devAddr24.data, sizeof(devAddr24.data));
	memcpy(out + sizeof (devAddr24.data), &payload, sizeof (payload));
	memcpy(out + sizeof (devAddr24.data) + sizeof (payload), MIC.data, sizeof (MIC.data));
}

void encode16Bit(struct encryptData* enc_data, uint8_t* in, uint8_t* out, time_t time) {
	uint24a_t newNe = {0};
	uint24a_t MIC = {0};

	newNe.ud = (MS2S(time) - enc_data->initTime) / EPOCH_DURATION_TIME;

	if (newNe.ud != enc_data->Ne.ud) {
		enc_data->Ne = newNe;

		enc_data->Ke = getKe(enc_data->Ka, enc_data->Ne);
		enc_data->Km = getKm(enc_data->Ka, enc_data->Ne);
		enc_data->DevAddr = getDevAddr(enc_data->Ka, enc_data->Ne);
		enc_data->Nn = 0;
	} else {
		enc_data->Nn = S2M((MS2S(time) - enc_data->initTime) % EPOCH_DURATION_TIME);
	}


	int16_t payload = cryptoMacPayload16(*(uint16_t*)in, enc_data->Ke, enc_data->Nn);
	MIC = getMIC16(enc_data->Km, enc_data->DevAddr, payload, enc_data->Nn);

	memcpy(out, enc_data->DevAddr.data, sizeof (enc_data->DevAddr.data));
	memcpy(out + sizeof (enc_data->DevAddr.data), &payload, sizeof (payload));
	memcpy(out + sizeof (enc_data->DevAddr.data) + sizeof (payload), MIC.data, sizeof (MIC.data));
}

void encode48Bit(struct encryptData* enc_data, uint8_t* in, uint8_t* out, time_t time) {
	uint24a_t newNe = {0};
	uint24a_t MIC = {0};

	newNe.ud = (MS2S(time) - enc_data->initTime) / EPOCH_DURATION_TIME;

	if (newNe.ud != enc_data->Ne.ud) {
		enc_data->Ne = newNe;

		enc_data->Ke = getKe(enc_data->Ka, enc_data->Ne);
		enc_data->Km = getKm(enc_data->Ka, enc_data->Ne);
		enc_data->DevAddr = getDevAddr(enc_data->Ka, enc_data->Ne);
		enc_data->Nn = 0;
	} else {
		enc_data->Nn = S2M((MS2S(time) - enc_data->initTime) % EPOCH_DURATION_TIME);
	}

	uint48a_t payload = cryptoMacPayload48(*(uint48a_t*)in, enc_data->Ke, enc_data->Nn);
	MIC = getMIC48(enc_data->Km, enc_data->DevAddr, payload, enc_data->Nn);

	memcpy(out, enc_data->DevAddr.data, sizeof (enc_data->DevAddr.data));
	memcpy(out + sizeof (enc_data->DevAddr.data), payload.data, sizeof (payload.data));
	memcpy(out + sizeof (enc_data->DevAddr.data) + sizeof (payload.data), MIC.data, sizeof (MIC.data));
}


uint128_256_t aes128Enc(uint128_256_t key, uint128_256_t data);
uint128_256_t aesCTR(uint128_256_t key, iv_t iv, uint128_256_t data);
uint128_256_t aesCTR_a(uint128_256_t key, iv_t iv, uint8_t* data, size_t size);
uint128_256_t aesECB(uint128_256_t key, uint128_256_t data);
uint24a_t aesCMAC(uint128_256_t key, uint8_t* data, size_t size);

uint128_256_t encECB(uint128_256_t key, uint128_256_t data);
uint128_256_t encCTR(uint128_256_t key, iv_t iv, uint128_256_t data);
uint24a_t encCMAC(uint128_256_t key, uint8_t* data, size_t size);


uint128_256_t getKa(uint128_256_t K0, uint16_t Na) {
    uint128_256_t ret = { 0 };
    iv_t iv = {0};

    //Na || 00..00
    memcpy((char*)&iv + sizeof(iv.data) - sizeof(Na), &Na, sizeof(Na));
    uint128_256_t t = {0};

    ret = encCTR(K0, iv, t);

    return ret;
}


uint24a_t getDevAddr(uint128_256_t Ka, uint24a_t Ne) {
    uint24a_t ret;
    uint128_256_t tmp;
    uint128_256_t tmpRet = { 0 };

    // 0x01 || Ne || 00..00
    memset(tmp.data, 0, sizeof(tmp.data));
    tmp.data[sizeof(tmp.data) - 1] = 0x01;
    memcpy(tmp.data + sizeof(tmp.data) - sizeof(Ne) - 1, &Ne, sizeof(Ne));

    tmpRet = encECB(Ka, tmp);

    memcpy(ret.data, tmpRet.data + sizeof(tmpRet.data) - sizeof(ret.data), sizeof(ret.data));

    return ret;
}

uint128_256_t getKm(uint128_256_t Ka, uint24a_t Ne) {
    uint128_256_t ret = { 0 };

    // 0x02 || Ne || 00..00
    iv_t iv = {0};
    iv.data[sizeof(iv) - 1] = 0x02;
    memcpy(iv.data + sizeof(iv.data) - sizeof(Ne) - 1, &Ne, sizeof(Ne));

    uint128_256_t t = {0};
    ret = encCTR(Ka, iv, t);

    return ret;
}

uint128_256_t getKe(uint128_256_t Ka, uint24a_t Ne) {
    uint128_256_t ret = { 0 };

    // 0x02 || Ne || 00..00
    iv_t iv = {0};
    iv.data[sizeof(iv) - 1] = 0x03;
    memcpy(iv.data + sizeof(iv.data) - sizeof(Ne) - 1, &Ne, sizeof(Ne));

    uint128_256_t t = {0};
    ret = encCTR(Ka, iv, t);

    return ret;
}

uint16_t cryptoMacPayload16(uint16_t macPayload, uint128_256_t Ke, uint16_t Nn) {
    uint16_t ret;
    iv_t iv = {0};
    uint128_256_t tmpRet = { 0 };

    //Nn || 00..00
    memcpy((char*)&iv + sizeof(iv.data) - sizeof(Nn), &Nn, sizeof(Nn));

    uint128_256_t t = {0};
    memcpy(t.data, &macPayload, sizeof(macPayload));

    tmpRet = encCTR(Ke, iv, t);

    // MSB
    ret = (tmpRet.data[1] << 8) | (tmpRet.data[0]);

    return ret;
}

uint48a_t cryptoMacPayload48(uint48a_t macPayload, uint128_256_t Ke, uint16_t Nn) {
    uint48a_t ret;
    iv_t iv = {0};
    uint128_256_t tmpRet = { 0 };

    // Ne || 00..00
    memcpy((char*)&iv + sizeof(iv.data) - sizeof(Nn), &Nn, sizeof(Nn));

    uint128_256_t t = {0};
    memcpy(t.data, macPayload.data, sizeof(macPayload.data));
    tmpRet = encCTR(Ke, iv, t);

    memcpy(ret.data, tmpRet.data, sizeof(ret.data));

    return ret;
}

uint24a_t getMIC16(uint128_256_t Km, uint24a_t DevAddr, uint16_t cryptoMacPayload, uint16_t Nn) {
    uint24a_t ret = { 0 };

    uint128a_t P;

    // 00..00
    memset(P.data, 0, sizeof(P.data));
    // DevAddr || 00..00
    memcpy(P.data + sizeof(P.data) - sizeof(DevAddr.data), DevAddr.data, sizeof(DevAddr.data));
    // DevAddr || cryptoMacPayload || 00..00
    memcpy(P.data + sizeof(P.data) - sizeof(DevAddr.data) - sizeof(cryptoMacPayload), &cryptoMacPayload, sizeof(cryptoMacPayload));
    // DevAddr || cryptoMacPayload || Nn || 00..00
    memcpy(P.data + sizeof(P.data) - sizeof(DevAddr.data) - sizeof(cryptoMacPayload) - sizeof(Nn), &Nn, sizeof(Nn));
    // DevAddr || cryptoMacPayload || Nn || 00..00 || 0x10
    P.data[0] = 0x10;

    uint128_256_t R;
    uint128_256_t K1;
    uint128_256_t t = {0};

    R = aesECB(Km, t);

    uint8_t msb = (R.data[sizeof(R.data) - 1] >> 7) & 1;

    // R << 1
    for (int i = sizeof(R.data) - 1; i >= 0; i--) {
        R.data[i] = R.data[i] << 1;
        if (i != 0)
            R.data[i] |= (R.data[i - 1] >> 7) & 1;
    }

    // if ( MSB1(R) = 1)
    if (msb) {
        uint128_256_t B = {0};
        B.data[0] = 0b10000111;

        for (unsigned int i = 0; i < sizeof(K1.data); i++) {
            K1.data[i] = R.data[i] ^ B.data[i];
        }
    }
    // if ( MSB1(R) = 0)
    else {
        for (unsigned int i = 0; i < sizeof(K1.data); i++) {
            K1.data[i] = R.data[i];
        }
    }

    for (unsigned int i = 0; i < sizeof(P.data); i++) {
        K1.data[i] ^= P.data[i];
    }

    memset(t.data, sizeof(t.data), 0);

    uint128_256_t tmp;

    tmp = aesECB(Km, K1);

    for (int i = 0; i < sizeof(ret.data); i++)
        ret.data[i] = tmp.data[sizeof(tmp.data) - sizeof(ret.data) + i];

    return ret;
}

uint24a_t getMIC48(uint128_256_t Km, uint24a_t DevAddr, uint48a_t cryptoMacPayload, uint16_t Nn) {
    uint24a_t ret = { 0 };

    uint128a_t P;

    // 00..00
    memset(P.data, 0, sizeof(P.data));
    // DevAddr || 00..00
    memcpy(P.data + sizeof(P.data) - sizeof(DevAddr.data), DevAddr.data, sizeof(DevAddr.data));
    // DevAddr || cryptoMacPayload || 00..00
    memcpy(P.data + sizeof(P.data) - sizeof(DevAddr.data) - sizeof(cryptoMacPayload.data), cryptoMacPayload.data, sizeof(cryptoMacPayload).data);
    // DevAddr || cryptoMacPayload || Nn || 00..00
    memcpy(P.data + sizeof(P.data) - sizeof(DevAddr.data) - sizeof(cryptoMacPayload.data) - sizeof(Nn), &Nn, sizeof(Nn));
    // DevAddr || cryptoMacPayload || Nn || 00..00 || 0x10
    P.data[0] = 0x30;


    uint128_256_t R;
    uint128_256_t K1;
    uint128_256_t t = {0};

    R = aesECB(Km, t);

    uint8_t msb = (R.data[sizeof(R.data) - 1] >> 7) & 1;

    // R << 1
    for (int i = sizeof(R.data) - 1; i >= 0; i--) {
        R.data[i] = R.data[i] << 1;
        if (i != 0)
            R.data[i] |= (R.data[i - 1] >> 7) & 1;
    }

    // if ( MSB1(R) = 1)
    if (msb) {
        uint128_256_t B = {0};
        B.data[0] = 0b10000111;

        for (unsigned int i = 0; i < sizeof(K1.data); i++) {
            K1.data[i] = R.data[i] ^ B.data[i];
        }
    }
    // if ( MSB1(R) = 0)
    else {
        for (unsigned int i = 0; i < sizeof(K1.data); i++) {
            K1.data[i] = R.data[i];
        }
    }

    for (unsigned int i = 0; i < sizeof(P.data); i++) {
        K1.data[i] ^= P.data[i];
    }

    memset(t.data, sizeof(t.data), 0); //t = {0};
    uint128_256_t tmp;

    tmp = aesECB(Km, K1);

    for (int i = 0; i < sizeof(ret.data); i++)
        ret.data[i] = tmp.data[sizeof(tmp.data) - sizeof(ret.data) + i];


    return ret;
}


uint128_256_t aes128Enc(uint128_256_t key, uint128_256_t data) {
    struct AES_ctx _key;
    //AES_KEY _key;
    uint128_256_t ret;
    memcpy(ret.data, data.data, sizeof(data.data));
    uint128_256_t iv = {0};

    AES_init_ctx_iv(&_key, key.data, iv.data);
    AES_CTR_xcrypt_buffer(&_key, ret.data, sizeof(ret.data));
    return ret;

}

uint128_256_t aesCTR(uint128_256_t key, iv_t iv, uint128_256_t data) {
    return aesCTR_a(key, iv, data.data, sizeof (data.data));
}

uint128_256_t aesCTR_a(uint128_256_t key, iv_t iv, uint8_t* data, size_t size) {
    struct AES_ctx _key;
    uint128_256_t ret;
    memcpy(ret.data, data, size);
    AES_init_ctx_iv(&_key, key.data, (uint8_t*)&iv);
    AES_CTR_xcrypt_buffer(&_key, ret.data, size);

    return ret;
}

uint128_256_t aesECB(uint128_256_t key, uint128_256_t data) {

    struct AES_ctx _key;
    uint128_256_t ret;
    memcpy(ret.data, data.data, sizeof(ret.data));
    AES_init_ctx(&_key, key.data);
    AES_ECB_encrypt(&_key, ret.data);


    return ret;
}

uint24a_t aesCMAC(uint128_256_t key, uint8_t* data, size_t size) {
	uint24a_t ret;
	ret.ud = 0;
    return ret;
}



uint128_256_t encECB(uint128_256_t key, uint128_256_t data) {
    uint128_256_t ret;

    ret = aesECB(key, data);//aes128Enc(Ka, tmp);

    return ret;
}

uint128_256_t encCTR(uint128_256_t key, iv_t iv, uint128_256_t data) {
    uint128_256_t ret;

    ret = aesCTR(key, iv, data);


    return ret;
}
