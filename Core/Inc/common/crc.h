

#ifndef __CRC_H_
#define __CRC_H_

#include "stdint.h"

uint8_t crc_cal(uint8_t *data, uint16_t len);
uint8_t reverse(uint8_t data);
uint8_t tmc_crc(uint8_t *data,uint8_t len);


#endif

