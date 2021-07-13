#include <stdint.h>

#define USE64
#define USE96

void init_polar();

void byties_to_bits(uint8_t* in, int size, uint8_t* out);
void bits_to_byties(uint8_t* in, int size, uint8_t* out);

#ifdef USE64
void encode64(uint8_t* in, uint8_t* out);
#endif

#ifdef USE96
void encode96(uint8_t* in, uint8_t* out);
#endif

void stdpolar_encode_systematic_noperm(uint8_t* _iwd, uint16_t _iwd_size, const uint8_t* _frozen_indicator, uint16_t _frozen_indicator_size, uint8_t* ret);
void solve_recursively(uint16_t* _inf_idx, uint16_t _inf_idx_size, uint8_t* _u, int uSize, uint8_t* _x, int xSize, int last);
void add_crc(uint32_t polynom, uint8_t* a, uint16_t size);
