#include "phy.h"

void to_diff(uint8_t* src, uint8_t* dis, uint16_t len) {
	
	uint8_t res = src[0] & 1;
	
	for (int i=0; i<len; i++) {
		uint8_t dis_p = 0;
		
		for (int j=0; j<8; j++) {
			if (j != 0 || i != 0)
				res = (res ^ ((src[i] >> j) & 1));
			dis_p |= res << j;
		}
		
		dis[i] = dis_p;
	}
	
}