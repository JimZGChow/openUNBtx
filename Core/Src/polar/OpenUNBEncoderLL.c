#include "polar/OpenUNBEncoderLL.h"

#ifdef USE64

const uint8_t frozen_indicator_64[] = {0,0,0,0,0,0,0,1,0,0,0,1,0,1,1,1,0,0,0,0,0,0,1,1,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,0,0,0,1,0,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,0,1,1,1,0,0,0,1,0,1,1,1,0,1,1,1,1,1,1,1,0,0,0,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
const int num_of_nonzero_bits_64 = 74;
const int short_64 = 0;

#endif
#ifdef USE96

const uint8_t frozen_indicator_96[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,1,0,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,0,0,0,1,0,1,1,1,0,0,0,1,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,1,1,0,1,1,1,1,1,1,1,0,0,0,1,0,1,1,1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
const int num_of_nonzero_bits_96 = 170;
#define short_96 64

#endif

void init_polar() {

}

void byties_to_bits(uint8_t* in, int size, uint8_t* out) {
	uint16_t i=0;

	for (; i < size; i++) {
		uint8_t bit = 0;

		for (; bit < 8; bit++) {
			out[i * 8 + bit] = (in[i] >> bit) & 1;
		}
	}
}

void bits_to_byties(uint8_t* in, int size, uint8_t* out) {
	uint16_t i=0;

	memset(out, 0, size/8);

	for (; i < size; i++) {
		out[i / 8] |= in[i] << (i % 8);
	}
}

#ifdef USE96

void encode96(uint8_t* in, uint8_t* out) {
	uint8_t data_bits[96 + 10 + short_96];
	memset(data_bits, 0, sizeof(data_bits));

	uint8_t out_bits[256];


	byties_to_bits(in, 12, data_bits);
	add_crc(0x327, data_bits, 96);

	stdpolar_encode_systematic_noperm(data_bits, 106, frozen_indicator_96, sizeof(frozen_indicator_96), out_bits);

	bits_to_byties(out_bits, 256, out);
}

#endif

#ifdef USE64
void encode64(uint8_t* in, uint8_t* out) {
	uint8_t data_bits[64 + 10];
	uint8_t out_bits[128];

	byties_to_bits(in, 8, data_bits);
	add_crc(0x327, data_bits, 64);

	stdpolar_encode_systematic_noperm(data_bits, 74, frozen_indicator_64, sizeof(frozen_indicator_64), out_bits);

	bits_to_byties(out_bits, 128, out);
}

#endif

void stdpolar_encode_systematic_noperm(uint8_t* _iwd, uint16_t _iwd_size, const uint8_t* _frozen_indicator, uint16_t _frozen_indicator_size, uint8_t* ret) {

    int n_polar = _frozen_indicator_size;
    uint8_t* iwd_s = ret;
    //uint16_t* inf_idx = (uint16_t*)malloc(n_polar * sizeof(uint16_t));//[256];// = (uint16_t*) malloc (sizeof(uint16_t) * 256);

	uint16_t inf_idx[256] = {0};
    uint8_t u[256] = {0};// = (uint8_t*) malloc (sizeof(uint8_t) * n_polar);


    uint16_t i = 0, j = 0;
    for (; i < n_polar; i++) {
        if (_frozen_indicator[i] == 1) {
            iwd_s[i] = _iwd[j];
            inf_idx[j] = i + 1;
            j++;
        }

        else {
            iwd_s[i] = 0;
        }
        u[i] = 0;
    }

    solve_recursively(inf_idx, j, u, n_polar, iwd_s, n_polar, n_polar);
		//free(inf_idx);
}


void solve_recursively(uint16_t* _inf_idx, uint16_t _inf_idx_size, uint8_t* _u, int uSize, uint8_t* _x, int xSize, int last) {
    int n = xSize;

    if (n == 1) {
        uint8_t f = 0;
        for (int i = 0; i < _inf_idx_size && !f; i++) {
            if (_inf_idx[i] == last) {
                f = 1;
            }
        }

        if (f) {
            *_u = *_x;
        }
        else {
            *_x = *_u;
        }
    }
    else {
        int n0 = n / 2;

        solve_recursively(_inf_idx, _inf_idx_size, _u + n0, uSize - n0, _x + n0, xSize - n0, last);

        //uint8_t* x_first = (uint8_t*)malloc(n0 * sizeof(uint8_t));
		uint8_t x_first[128];

        for (int i = 0; i < n0; i++) {
            x_first[i] = _x[i] ^ _x[i + n0];
        }

        solve_recursively(_inf_idx, _inf_idx_size, _u, n0, x_first, n0, last - n0);

        for (int i = 0; i < n0; i++) {
            _x[i] = _x[n0 + i] ^ x_first[i];
        }

        //free(x_first);

    }
}


void add_crc(uint32_t polynom, uint8_t* a, uint16_t size) {
    uint32_t reg = 0;
    for (int i = 0; i < size; i++) {
        reg ^= a[i];
        if (reg & 1) reg = (reg >> 1) ^ polynom;
        else reg >>= 1;
    }
    reg = reg & 0x3FF;

    for (int i = 0; i < 10; i++)
        a[size + i] = (reg >> i) & 1;
}
