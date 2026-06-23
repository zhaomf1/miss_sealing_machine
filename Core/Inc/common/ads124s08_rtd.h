/**
 ******************************************************************************
 * @file    ads124s08_rtd.h
 * @brief   ADS124S08 24-bit ADC driver for PT100 RTD temperature measurement
 *
 *          Three-wire ratiometric measurement with dual IDAC current sources.
 *          基于已验证的 yizhi 版本, 适配当前硬件引脚.
 ******************************************************************************
 */

#ifndef __ADS124S08_RTD_H__
#define __ADS124S08_RTD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "gpio.h"
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include <math.h>

/* ===================================================================
 *        引脚定义 (匹配 MISS FengMoJiKZB-V1.0 GPIO LIST)
 *        SPI1 连接 ADS124S08
 * =================================================================== */
#define CS_PORT         GPIOA       /* SPI1 手动片选 (PA4)          */
#define CS_PIN          GPIO_PIN_4
#define START_PORT      GPIOC       /* 转换启动 (PC0, 高电平有效)   */
#define START_PIN       GPIO_PIN_0
#define RST_PORT        GPIOC       /* 硬件复位 (PC1, 低电平有效)   */
#define RST_PIN         GPIO_PIN_1
#define DRDY_PORT       GPIOC       /* 数据就绪 (PC2, 低电平有效)   */
#define DRDY_PIN        GPIO_PIN_2

/* 引脚控制宏 */
#define CS_LOW()        HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH()       HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET)
#define START_LOW()     HAL_GPIO_WritePin(START_PORT, START_PIN, GPIO_PIN_RESET)
#define START_HIGH()    HAL_GPIO_WritePin(START_PORT, START_PIN, GPIO_PIN_SET)
#define RST_LOW()       HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_RESET)
#define RST_HIGH()      HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_SET)
#define DRDY_IS_LOW()   (HAL_GPIO_ReadPin(DRDY_PORT, DRDY_PIN) == GPIO_PIN_RESET)

/* ===================================================================
 *        硬件参数 (严格匹配电路图)
 * =================================================================== */
#define IDAC_CURRENT    0.001f      /* IDAC 电流 1mA                */
#define PGA_GAIN        8           /* PGA 增益 8V/V (REG_PGA=0x0B) */
#define RTD_R0          100.0f      /* PT100 0°C 电阻 (Ω)           */
#define R_REF           820.0f      /* 参考电阻 (Ω)                 */

/* ===================================================================
 *        PT100 Callendar-Van Dusen 系数
 * =================================================================== */
#define A               3.9083e-3f  /* 线性系数                     */
#define B               -5.775e-7f  /* 二次项系数 (0°C 以上)        */
#define C               -4.183e-12f /* 四次项系数 (0°C 以下)        */

/* 别名 (兼容旧代码) */
#define CVD_A           A
#define CVD_B           B
#define CVD_C           C

/* ===================================================================
 *        SPI 命令 (芯片手册)
 * =================================================================== */
#define CMD_RESET       0x06        /* 软件复位                     */
#define CMD_START       0x08        /* 启动转换                     */
#define CMD_STOP        0x0A        /* 停止转换                     */
#define CMD_RREG        0x20        /* 读寄存器 (| 寄存器地址)      */
#define CMD_WREG        0x40        /* 写寄存器 (| 寄存器地址)      */
#define CMD_RDATA       0x12        /* 读转换数据                   */

/* ===================================================================
 *        寄存器地址
 * =================================================================== */
#define REG_ID          0x00        /* 器件 ID (只读)               */
#define REG_STATUS      0x01        /* 状态寄存器 (只读)            */
#define REG_INPMUX      0x02        /* 输入多路复用器               */
#define REG_PGA         0x03        /* PGA 增益配置                 */
#define REG_DATARATE    0x04        /* 数据速率 / 转换模式          */
#define REG_REF         0x05        /* 基准电压配置                 */
#define REG_IDACMAG     0x06        /* IDAC 电流幅度                */
#define REG_IDACMUX     0x07        /* IDAC 输出通道选择            */
#define REG_VBIAS       0x08        /* 偏置电压配置                 */
#define REG_SYS         0x09        /* 系统控制                     */

/* ===================================================================
 *        函数声明
 * =================================================================== */
void  ads124s08_hard_reset(void);
int   ads124s08_init(SPI_HandleTypeDef *hspi);
void  ads124s08_select_sensor(uint8_t sensor);
float ads124s08_read_temperature(SPI_HandleTypeDef *hspi);
float ads124s08_read_sensor_temp(SPI_HandleTypeDef *hspi, uint8_t sensor);
void  ads124s08_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg,
                         uint8_t *data, uint8_t len);
void  ads124s08_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg,
                          uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __ADS124S08_RTD_H__ */
