#ifndef __DEV_OD_CTRL_H
#define __DEV_OD_CTRL_H

#include <stdint.h>

typedef enum{
    SHUTTER_OFF = 0,
    SHUTTER_ON
}ShutterState_t;

typedef enum{
    SHUTTER1_ADDR = 0x0004,
    SHUTTER2_ADDR = 0x0005
}ShutterAddr_t;

/**
 * @brief 读取当前OD值
 * @param slave 从机地址
 * @param temperature 输出温度值 (单位 0.01°C，例如 2530 表示 25.30°C)
 * @return MODBUS_OK 成功，其他错误码
 */
int od_ctrl_read_value(uint16_t *od_data);
int od_ctrl_set_shutter1_open(void);
int od_ctrl_set_shutter1_off(void);
int od_ctrl_set_shutter2_open(void);
int od_ctrl_set_shutter2_off(void);

#endif
