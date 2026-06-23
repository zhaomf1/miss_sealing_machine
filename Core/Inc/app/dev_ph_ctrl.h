#ifndef __DEV_PH_CTRL_H
#define __DEV_PH_CTRL_H

#include <stdint.h>

// 定义PH测量数据结构体（对应10个寄存器/20字节）
typedef struct {
    uint16_t unit_high;       // 物理单位高16位 (Reg1)
    uint16_t unit_low;        // 物理单位低16位 (Reg2)
    uint16_t ph_value_high;   // PH值高16位 (Reg3)
    uint16_t ph_value_low;    // PH值低16位 (Reg4)
    uint16_t status_high;     // 测量状态高16位 (Reg5)
    uint16_t status_low;      // 测量状态低16位 (Reg6)
    uint16_t ph_min_high;     // PH最小值高16位 (Reg7)
    uint16_t ph_min_low;      // PH最小值低16位 (Reg8)
    uint16_t ph_max_high;     // PH最大值高16位 (Reg9)
    uint16_t ph_max_low;      // PH最大值低16位 (Reg10)
} PH_RegisterData;

// 异常状态定义（便于解析）
typedef enum {
    STATUS_NORMAL = 0x0000,          // 正常
    STATUS_TEMP_OUT_MEASURE = 0x0001,// 温度超出测量范围
    STATUS_TEMP_OUT_WORK = 0x0002,   // 温度超出工作范围
    STATUS_CALIBRATION_ERR = 0x0004, // 校准异常
    STATUS_WARNING = 0x0008,         // 警告
    STATUS_ERROR = 0x0010            // 错误（电极故障等）
} PH_MeasureStatus;

/**
 * @brief 读取当前ph值
 * @param od ph值
 * @return MODBUS_OK 成功，其他错误码
 */
int ph_ctrl_read_value(float *od);

#endif
