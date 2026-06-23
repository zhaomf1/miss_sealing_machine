#ifndef __AT24C02_H
#define __AT24C02_H

#include "main.h"
#include "i2c.h"

HAL_StatusTypeDef at24c02_write_byte(uint8_t addr, uint8_t data);
HAL_StatusTypeDef at24c02_read_byte(uint8_t addr, uint8_t *data);
HAL_StatusTypeDef at24c02_write_page(uint8_t addr, uint8_t *data, uint8_t len);
HAL_StatusTypeDef at24c02_read_buffer(uint8_t addr, uint8_t *data, uint16_t len);

#endif
