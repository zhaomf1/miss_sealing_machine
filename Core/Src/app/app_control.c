#include <stdio.h>
#include <math.h>
#include "app_control.h"
#include "cmsis_os2.h"
#include "i2c.h"
#include "at24c02.h"
#include "ads124s08_rtd.h"
#include "spi.h"
#include "modbus_slave_reg.h"

/* 温控模块硬件定义 (匹配 MISS FengMoJiKZB-V1.0 GPIO LIST)
 * PA15 = OUTPUT_1: 加热控制，低电平有效
 *   低电平 → 加热器输出24V (开), LED402亮
 *   高电平 → 加热器输出0V  (关), LED402灭
 */
#define HEATER_PORT   GPIOA
#define HEATER_PIN    GPIO_PIN_15


/* ===================================================================
 *              EEPROM 读写接口 (AT24C02, I2C1)
 *
 *  每参数占 4 字节（uint32_t），写操作自动处理 8 字节页边界。
 * =================================================================== */

/**
 * @brief 从 EEPROM 指定地址读取 uint32_t 数据
 * @param addr EEPROM 内部地址（使用 EEPROM_ADDR_* 宏）
 * @param data 输出数据指针
 * @return HAL_OK 成功
 */
HAL_StatusTypeDef eeprom_read_u32(uint16_t addr, uint32_t *data)
{
    uint8_t buf[4];
    HAL_StatusTypeDef ret;

    if (data == NULL) return HAL_ERROR;
    if (addr + 4 > 255) return HAL_ERROR;

    ret = at24c02_read_buffer((uint8_t)addr, buf, 4);
    if (ret != HAL_OK) return ret;

    *data = ((uint32_t)buf[0] << 0)
          | ((uint32_t)buf[1] << 8)
          | ((uint32_t)buf[2] << 16)
          | ((uint32_t)buf[3] << 24);
    return HAL_OK;
}

/**
 * @brief 向 EEPROM 指定地址写入 uint32_t 数据
 * @param addr EEPROM 内部地址（使用 EEPROM_ADDR_* 宏）
 * @param data 要写入的数据
 * @return HAL_OK 成功
 */
HAL_StatusTypeDef eeprom_write_u32(uint16_t addr, uint32_t data)
{
    uint8_t buf[4];

    if (addr + 4 > 255) return HAL_ERROR;

    buf[0] = (uint8_t)(data >> 0);
    buf[1] = (uint8_t)(data >> 8);
    buf[2] = (uint8_t)(data >> 16);
    buf[3] = (uint8_t)(data >> 24);

    return at24c02_write_page((uint8_t)addr, buf, 4);
}


/* ===================================================================
 *              温度控制
 *
 * =================================================================== */

/* ================================================
   内部变量
   ================================================ */
static volatile uint8_t g_temp_ctrl_enabled = 0; // 温控使能标志，默认关闭
static float g_setpoint = 180.0f;           // 设定温度
static volatile float g_current_temp = 0.0f; // 最近一次有效实时温度

/* PID 参数 */
static float Kp = 4.0f;
static float Ki = 0.08f;
static float Kd = 0.3f;

static float error;
static float last_error;
static float integral;
static float derivative;
static float pid_output;

static const float output_max = 100.0f;
static const float output_min = 0.0f;

static const uint32_t pwm_period = 2000;    // 毫秒
static uint32_t heat_on_time;
static uint8_t  heating_flag = 0;
static uint32_t heat_start_time;

static const float feedforward_threshold = 20.0f;
static const float anti_windup_threshold = 10.0f;

/* ================================================
   GPIO 驱动封装
   ================================================ */

static void heart_on(void)
{
    HAL_GPIO_WritePin(HEATER_PORT, HEATER_PIN, GPIO_PIN_SET);  // 低电平有效: 开加热
}

static void heart_off(void)
{
    HAL_GPIO_WritePin(HEATER_PORT, HEATER_PIN, GPIO_PIN_RESET);    // 高电平: 关加热
}

/* ================================================
   公共接口：获取/设置设定温度
   ================================================ */

void PID_SetTemperature(float setpoint)
{
    g_setpoint = setpoint;
}

float PID_GetSetpoint(void)
{
    return g_setpoint;
}

float PID_GetCurrentTemperature(void)
{
    return g_current_temp;
}

/* ================================================
   温控开关接口
   ================================================ */

/**
 * @brief 开启温度控制
 */
void temp_ctrl_start(void)
{
    g_temp_ctrl_enabled = 1;
    printf("[TEMP] Temperature control started, setpoint=%.0fC\r\n", g_setpoint);
}

/**
 * @brief 关闭温度控制（立即停止加热）
 */
void temp_ctrl_stop(void)
{
    g_temp_ctrl_enabled = 0;
    heart_off();
    heating_flag = 0;
    printf("[TEMP] Temperature control stopped\r\n");
}

/**
 * @brief 查询温控是否运行中
 * @return 1=运行, 0=停止
 */
uint8_t temp_ctrl_is_running(void)
{
    return g_temp_ctrl_enabled;
}


/**
 * @brief 温度控制任务 — 双路传感器采集 + PID控制
 */
void temp_ctrl_task(void *argument)
{
    /* 一次性初始化 ADS124S08 (SPI1), 失败则重试 */
    int ads_init_ret;
    for (int retry = 0; retry < 3; retry++) {
        ads_init_ret = ads124s08_init(&hspi1);
        if (ads_init_ret == 0)
        {
            printf("[TEMP] ADS124S08 init successful\r\n");
            break;
        }
        printf("[TEMP] ADS124S08 init failed, retry %d/3\r\n", retry + 1);
        HAL_Delay(500);
    }
    if (ads_init_ret != 0) {
        printf("[TEMP] FATAL: ADS124S08 init failed\r\n");
        modbus_reg_set_temp_fault(1);
        osThreadSuspend(NULL);
    }

    uint32_t last_print_tick = 0;

    for (;;)
    {
        uint32_t current_time = osKernelGetTickCount();

        /* 读取两路传感器 */
        float t1 = ads124s08_read_sensor_temp(&hspi1, 0);  /* 传感器1: AIN0~3 */
        float t2 = ads124s08_read_sensor_temp(&hspi1, 1);  /* 传感器2: AIN4~7 */
        modbus_reg_set_temp_fault((isnan(t1) || isnan(t2)) ? 1 : 0);

        /*
         * 温度采集与加热控制相互独立：
         * 无论温控开关是否开启，只要传感器1数据有效，就持续刷新0x0020实时温度。
         */
        if (!isnan(t1)) {
            g_current_temp = t1;
        }

        /* 无论温控是否开启，都定期输出实时采样值 */
        if (current_time - last_print_tick >= 10000) {
            last_print_tick = current_time;
            printf("T1=%.1fC  T2=%.1fC  Set=%.0fC  Ctrl=%s\r\n",
                   isnan(t1) ? -999.0f : t1,
                   isnan(t2) ? -999.0f : t2,
                   g_setpoint,
                   g_temp_ctrl_enabled ? "ON" : "OFF");
        }

        /* 温控未使能时，关闭加热并等待 */
        if (!g_temp_ctrl_enabled) {
            heart_off();
            heating_flag = 0;
            osDelay(200);
            continue;
        }

        /* PID 控制 (传感器1) */
        if (isnan(t1))
        {
            heart_off();
            heating_flag = 0;
            osDelay(100);
            continue;
        }

        error = g_setpoint - g_current_temp;

        if (error > feedforward_threshold)
        {
            pid_output = output_max;
            integral = 0.0f;
        }
        else
        {
            if (error < anti_windup_threshold && error > -anti_windup_threshold)
                integral += error * 0.5f;

            if (integral > 100.0f) integral = 100.0f;
            if (integral < -100.0f) integral = -100.0f;

            derivative = error - last_error;
            pid_output = Kp * error + Ki * integral + Kd * derivative;

            if (pid_output > output_max) pid_output = output_max;
            if (pid_output < output_min) pid_output = output_min;
        }
        last_error = error;

        heat_on_time = (uint32_t)(pid_output * pwm_period / 100.0f);

        if (pid_output >= output_max - 1.0f)
        {
            heart_on();
            heating_flag = 0;
        }
        else if (pid_output <= output_min + 1.0f)
        {
            heart_off();
            heating_flag = 0;
        }
        else if (heating_flag == 0)
        {
            heating_flag = 1;
            heat_start_time = current_time;
            heart_on();
        }
        else
        {
            if (heating_flag == 1 && (current_time - heat_start_time) >= heat_on_time)
            {
                heart_off();
                heating_flag = 2;
            }
            if ((current_time - heat_start_time) >= pwm_period)
                heating_flag = 0;
        }

        osDelay(100);
    }
}


/**
 * @brief 上电指示灯闪烁
 */
void led_blink(void)
{
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
}
