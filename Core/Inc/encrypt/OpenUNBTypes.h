#ifndef OPENUNBTYPES_H
#define OPENUNBTYPES_H

#include <stdint.h>
#include <time.h>

#define AES128

#ifdef AES128
#define KEYSIZE 128
#else
#define KEYSIZE 256
#endif

#if defined(MAGMA)
struct iv_t {
    uint8_t data[4];
};
#elif defined(KUZNECHIK)
struct iv_t {
    uint8_t data[8];
};
#else
struct iv_t {
    uint8_t data[16];
};
#endif

struct uint256a_t {
    uint8_t data[32];
};

struct uint128a_t {
    uint8_t data[16];
};

union uint64a_t {
    uint8_t data[8];
    uint64_t ud;
};

union uint48a_t {
    uint8_t data[6];
    uint64_t ud : 48;
};

union uint24a_t {
    uint8_t data[3];
    uint32_t ud : 24;
};

#if KEYSIZE == 128
typedef struct uint128a_t uint128_256_t;
#else
typedef uint256a_t uint128_256_t;
#endif

typedef union uint24a_t uint24a_t;
typedef union uint48a_t uint48a_t;
typedef struct uint128a_t uint128a_t;
typedef struct uint256a_t uint256a_t;
typedef struct iv_t iv_t;


struct encryptData {
    uint8_t* DevID;
    uint8_t DevID_len;
    uint128_256_t K0;
    uint16_t Na;
    uint24a_t Ne;

    uint128_256_t Ka;
    uint24a_t DevAddr;
    uint128_256_t Km;
    uint128_256_t Ke;

    uint16_t Nn;
    time_t initTime;
    //time_t epocheInitTime;
};

#endif // OPENUNBTYPES_H
