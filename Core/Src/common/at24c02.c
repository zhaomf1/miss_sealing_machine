#include "i2c.h"


#define AT24C02_ADDR        0xA0
#define AT24C02_PAGE_SIZE   8
#define AT24C02_MAX_ADDR    255

/**
 * @brief AT24C02 写一个字节
 * @param addr 内存地址 (0-255)
 * @param data 要写入的数据
 * @return HAL_OK 成功，其他失败
 */
HAL_StatusTypeDef at24c02_write_byte(uint8_t addr, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR, addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

/**
 * @brief AT24C02 读一个字节
 * @param addr 内存地址 (0-255)
 * @param data 读取到的数据
 * @return HAL_OK 成功，其他失败
 */
HAL_StatusTypeDef at24c02_read_byte(uint8_t addr, uint8_t *data)
{
    return HAL_I2C_Mem_Read(&hi2c1, AT24C02_ADDR, addr, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

/**
 * @brief AT24C02 写一页数据（最多8字节）
 * @param addr 起始地址 (0-255)
 * @param data 要写入的数据指针
 * @param len 数据长度 (1-8)
 * @return HAL_OK 成功，其他失败
 */
HAL_StatusTypeDef at24c02_write_page(uint8_t addr, uint8_t *data, uint8_t len)
{
    if (len > AT24C02_PAGE_SIZE) len = AT24C02_PAGE_SIZE;
    return HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR, addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

/**
 * @brief AT24C02 连续读取多个字节
 * @param addr 起始地址 (0-255)
 * @param data 读取数据缓冲区
 * @param len 读取长度
 * @return HAL_OK 成功，其他失败
 */
HAL_StatusTypeDef at24c02_read_buffer(uint8_t addr, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, AT24C02_ADDR, addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}
