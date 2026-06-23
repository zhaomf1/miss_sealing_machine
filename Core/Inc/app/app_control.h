#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"


typedef enum{
    MODBUS_ADDR_SUCTION_CUP     = 0x01,             // 慧灵电动吸盘
    MODBUS_ADDR_SUCK_CYLINDER   = 0x02,             // 吸膜电缸
    MODBUS_ADDR_PAVE_CYLINDER   = 0x03,             // 铺膜电缸
    MODBUS_ADDR_FUJUN_MOTOR     = 0x04,             // 富俊步进电机

}ModbusAddr_t;              // Modbus 从机地址枚举

//EEPROM 地址
#define EEPROM_ADDR_ZERO        0x00   //归零点,放膜盒位置
#define EEPROM_SUCK_SEAL        0x04   //吸膜/封膜点
#define EEPROM_PAVE             0x08   //铺膜点
#define EEPROM_GET_PLACE        0x0C   //取放孔板点
#define EEPROM_TEMP_CTRL        0x10   //温控参数
#define EEPROM_PRESS_TIME       0x14   //压膜时间
#define EEPROM_FREQUENCY        0x18   //总封膜次数

//EEPROM 读写接口
HAL_StatusTypeDef eeprom_read_u32(uint16_t addr, uint32_t *data);
HAL_StatusTypeDef eeprom_write_u32(uint16_t addr, uint32_t data);

void temp_ctrl_task(void *argument);

/* 温控开关接口 */
void    temp_ctrl_start(void);          // 开启温度控制
void    temp_ctrl_stop(void);           // 关闭温度控制
uint8_t temp_ctrl_is_running(void);     // 查询温控运行状态 (1=运行, 0=停止)

/* 温度读取接口（供 modbus_slave_reg 调用） */
float PID_GetCurrentTemperature(void);
void  PID_SetTemperature(float setpoint);
float PID_GetSetpoint(void);
void led_blink(void);

#endif
