/**
 ******************************************************************************
 * @file    ads124s08_rtd.c
 * @brief   ADS124S08 双路 PT100 温度采集驱动
 *          传感器1: AIN0~AIN3, 传感器2: AIN4~AIN7
 ******************************************************************************
 */

#include "ads124s08_rtd.h"
#include <string.h>
#include <stdio.h>

/* 调试打印 (0=安静模式, 1=详细日志) */
#define ADS_VERBOSE 1

#if ADS_VERBOSE
#define ADS_LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define ADS_LOG(fmt, ...)
#endif

/* 当前选中的传感器 (0=AIN0~3, 1=AIN4~7) */
static uint8_t g_sensor = 0;

/* 传感器配置表: [INPMUX, IDACMUX_DEF, IDACMUX_SWAP] */
static const uint8_t SENSOR_CFG[2][3] = {
    /* INPMUX,   IDAC(默认), IDAC(对调) */
    {   0x12,     0x30,       0x03    },  /* 传感器1: AIN1/AIN2, IDAC→AIN0/AIN3 */
    {   0x56,     0x74,       0x47    },  /* 传感器2: AIN5/AIN6, IDAC→AIN4/AIN7 */
};

/* ===================================================================
 *       SPI 基础操作
 * =================================================================== */

static void write_cmd(SPI_HandleTypeDef *hspi, uint8_t cmd)
{
    CS_LOW();
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    CS_HIGH();
}

void ads124s08_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg,
                       uint8_t *data, uint8_t len)
{
    uint8_t cmd[2] = {CMD_RREG | reg, len - 1};
    CS_LOW();
    for (uint32_t i = 0; i < 100; i++) { __NOP(); }
    HAL_SPI_Transmit(hspi, cmd, 2, HAL_MAX_DELAY);
    for (uint32_t i = 0; i < 50; i++) { __NOP(); }
    HAL_SPI_Receive(hspi, data, len, HAL_MAX_DELAY);
    CS_HIGH();
}

void ads124s08_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg,
                        uint8_t *data, uint8_t len)
{
    uint8_t tx_buf[2 + len];
    tx_buf[0] = CMD_WREG | reg;
    tx_buf[1] = len - 1;
    memcpy(&tx_buf[2], data, len);
    CS_LOW();
    HAL_SPI_Transmit(hspi, tx_buf, 2 + len, HAL_MAX_DELAY);
    CS_HIGH();
}

/* 前置声明 (定义在下方, Init 中需要调用) */
static uint8_t  wait_drdy(void);
static int32_t  read_data(SPI_HandleTypeDef *hspi);

/* ===================================================================
 *       硬件复位
 * =================================================================== */

void ads124s08_hard_reset(void)
{
    RST_LOW();
    HAL_Delay(10);
    RST_HIGH();
    HAL_Delay(10);
    /* START 保持低电平, 转换完全由 CMD_START 软件触发,
       避免上电后意外产生一次无效转换导致首次读数错误 */
}

/* ===================================================================
 *       初始化 (上电一次调用)
 * =================================================================== */

int ads124s08_init(SPI_HandleTypeDef *hspi)
{
    uint8_t regs[8] = {
        0x12,   /* INPMUX:   默认传感器1 AIN1/AIN2               */
        0x0B,   /* PGA:      PGA_EN=1, GAIN=8                   */
        0x14,   /* DATARATE: Duty-Cycle模式(单次转换后待机),
                   DR=000, MODE=01, Sinc1滤波               */
        0x12,   /* REF:      外部 REFP0, 内部基准禁用            */
        0x07,   /* IDACMAG:  1mA IDAC 电流                      */
        0x30,   /* IDACMUX:  默认 IDAC1→AIN0, IDAC2→AIN3        */
        0x00,   /* VBIAS:    无偏置                             */
        0x10    /* SYS:      Sinc1 滤波                         */
    };

    ads124s08_hard_reset();
    write_cmd(hspi, CMD_RESET);
    HAL_Delay(100);

    uint8_t dev_id;
    ads124s08_read_reg(hspi, REG_ID, &dev_id, 1);
    if (dev_id != 0x08) {
        printf("[ADS] ERROR: Device ID 0x%02X != 0x08\r\n", dev_id);
        return -1;
    }

    ads124s08_write_reg(hspi, REG_INPMUX, regs, 8);

    /* START 拉高使能转换触发 (duty-cycle模式下 CMD_START 依赖此引脚为高) */
    printf("[ADS] Init: START HIGH, DRDY state=%d\r\n",
           HAL_GPIO_ReadPin(DRDY_PORT, DRDY_PIN));
    START_HIGH();
    HAL_Delay(5);

    /* 丢弃上电后首次自动转换的无效数据, 确保后续读数干净 */
    printf("[ADS] Init: after delay, DRDY state=%d\r\n",
           HAL_GPIO_ReadPin(DRDY_PORT, DRDY_PIN));
    if (wait_drdy()) {
        int32_t dummy = read_data(hspi);
        printf("[ADS] Init: dummy read code=%ld\r\n", dummy);
    } else {
        printf("[ADS] Init: dummy WaitDRDY TIMEOUT!\r\n");
    }

    ADS_LOG("[ADS] Init OK, ID=0x%02X\r\n", dev_id);
    g_sensor = 0;
    return 0;
}

/* ===================================================================
 *       传感器切换 (运行时切换 INPMUX + IDACMUX)
 * =================================================================== */

void ads124s08_select_sensor(uint8_t sensor)
{
    if (sensor > 1) return;
    g_sensor = sensor;
}

/* ===================================================================
 *       内部: 设置当前传感器的 IDAC 通道
 * =================================================================== */

static void config_idac(SPI_HandleTypeDef *hspi, uint8_t is_swap)
{
    uint8_t idac_mux = SENSOR_CFG[g_sensor][is_swap ? 2 : 1];
    ads124s08_write_reg(hspi, REG_IDACMUX, &idac_mux, 1);
    HAL_Delay(10);
}

/* ===================================================================
 *       DRDY 等待
 * =================================================================== */

static uint8_t wait_drdy(void)
{
    uint32_t tick_start = HAL_GetTick();
    uint8_t  drdy_initial = HAL_GPIO_ReadPin(DRDY_PORT, DRDY_PIN);
    while (HAL_GPIO_ReadPin(DRDY_PORT, DRDY_PIN) == GPIO_PIN_SET) {
        if (HAL_GetTick() - tick_start > 500) {
            printf("[ADS] DRDY TIMEOUT! initial=%d, current=%d, elapsed=%lu\r\n",
                   drdy_initial, HAL_GPIO_ReadPin(DRDY_PORT, DRDY_PIN),
                   HAL_GetTick() - tick_start);
            return 0;
        }
    }
    return 1;
}

/* ===================================================================
 *       ADC 数据读取 (24-bit → int32)
 * =================================================================== */

static int32_t read_data(SPI_HandleTypeDef *hspi)
{
    uint8_t data[3] = {0};
    CS_LOW();
    HAL_SPI_Transmit(hspi, (uint8_t[]){CMD_RDATA}, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, data, 3, HAL_MAX_DELAY);
    CS_HIGH();

    int32_t adc_val = (int32_t)((data[0] << 16) | (data[1] << 8) | data[2]);
    if (data[0] & 0x80)
        adc_val |= 0xFF000000;
    return adc_val;
}

/* ===================================================================
 *       电阻 → 温度
 * =================================================================== */

static float resistance_to_temperature(float Rt)
{
    if (Rt < 10.0f || Rt > 500.0f)
        return NAN;

    const float R0 = RTD_R0;
    float t = (Rt / R0 - 1.0f) / A;  /* 初始线性估计 */

    if (Rt >= R0) {
        /* T >= 0°C: Rt = R0*(1 + A*t + B*t²)
           牛顿迭代修正二次项, 否则 ~180°C 时误差 ~5°C */
        for (int i = 0; i < 5; i++) {
            t = (Rt / R0 - 1.0f - B * t * t) / A;
        }
    } else {
        /* T < 0°C: 完整 Callendar-Van Dusen 方程 Rt=R0*(1+A*t+B*t²+C*(t-100)*t³) */
        for (int i = 0; i < 5; i++) {
            t = (Rt / R0 - 1.0f - A * t - B * t * t - C * t * t * (t - 100.0f))
              / (A + 2.0f * B * t + 3.0f * C * t * t * (t - 100.0f));
        }
    }
    return t;
}

/* ===================================================================
 *       公开接口: 读取当前选中传感器的温度 (单次测量, 不做IDAC交换)
 * =================================================================== */

float ads124s08_read_temperature(SPI_HandleTypeDef *hspi)
{
    write_cmd(hspi, CMD_START);

    if (!wait_drdy()) {
        printf("[ADS] ReadTemp: WaitDRDY FAILED\r\n");
        return NAN;
    }

    int32_t code = read_data(hspi);

    /* R_RTD = code * 2 * R_REF / (PGA * 2^23)
       推导: code = (V_RTD*PGA/V_REF)*2^23, V_REF=2*I*R_REF, V_RTD=I*R_RTD */
    float r = (float)code / (float)(1UL << 23) * R_REF * 2.0f / (float)PGA_GAIN;

    float temp = resistance_to_temperature(r);
    return temp;
}

/* ===================================================================
 *       公开接口: 读取指定传感器的温度 (自动切换 INPMUX+IDACMUX)
 * =================================================================== */

float ads124s08_read_sensor_temp(SPI_HandleTypeDef *hspi, uint8_t sensor)
{
    if (sensor > 1) return NAN;
    g_sensor = sensor;


    /* 配置该传感器的 INPMUX */
    uint8_t inpmux = SENSOR_CFG[sensor][0];
    ads124s08_write_reg(hspi, REG_INPMUX, &inpmux, 1);
    HAL_Delay(1);  /* 等待输入 mux 和 PGA 建立 */

    /* ---- 第一次测量: 默认 IDAC 通道 ---- */
    config_idac(hspi, 0);
    float t1 = ads124s08_read_temperature(hspi);

    /* ---- 第二次测量: IDAC 通道对调 (消除三线制引线电阻误差) ---- */
    config_idac(hspi, 1);
    float t2 = ads124s08_read_temperature(hspi);

    /* 恢复默认 IDAC 配置 */
    config_idac(hspi, 0);

    /* 取平均消除引线电阻不对称误差 */
    if (!isnan(t1) && !isnan(t2))
        return (t1 + t2) / 2.0f;
    else
        return isnan(t1) ? t2 : t1;
}
