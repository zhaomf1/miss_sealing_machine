#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#include <stdint.h>
#include "cmsis_os2.h"

/* Modbus帧结构 */
typedef struct {
    uint8_t data[256];  // 数据缓冲区
    uint16_t len;       // 数据长度
} modbus_frame_t;

/* 功能码 */
#define MODBUS_FC_READ_COILS            0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS  0x02
#define MODBUS_FC_READ_HOLDING_REGS     0x03
#define MODBUS_FC_READ_INPUT_REGS       0x04
#define MODBUS_FC_WRITE_SINGLE_COIL     0x05
#define MODBUS_FC_WRITE_SINGLE_REG      0x06
#define MODBUS_FC_WRITE_MULTIPLE_REGS   0x10

/* 异常码 */
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION      0x01
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS  0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE    0x03
#define MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE  0x04
#define MODBUS_EXCEPTION_ACKNOWLEDGE           0x05
#define MODBUS_EXCEPTION_SLAVE_BUSY            0x06
#define MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE  0x07
#define MODBUS_EXCEPTION_MEMORY_PARITY_ERROR   0x08

/* 错误码 */
#define MODBUS_OK                   0
#define MODBUS_ERR_MUTEX            -1
#define MODBUS_ERR_SEND             -2
#define MODBUS_ERR_TIMEOUT          -3
#define MODBUS_ERR_CRC              -4
#define MODBUS_ERR_ADDR             -5
#define MODBUS_ERR_EXCEPTION        -6
#define MODBUS_ERR_RESP_LEN         -7
#define MODBUS_ERR_CREAT_MUTEX      -8
#define MODBUS_ERR_PARM             -9
#define MODBUS_ERR_SEM              -10
#define MODBUS_ERR_CHAR_TIMEOUT     -11

/* Modbus配置结构体 */
typedef struct {
    uint32_t timeout_ms;          /* 通信超时时间 */
    uint8_t retry_count;           /* 重试次数 */
} ModbusConfig_t;

/* Modbus状态结构体 */
typedef struct {
    uint32_t tx_count;             /* 发送次数 */
    uint32_t rx_count;             /* 接收次数 */
    uint32_t error_count;          /* 错误次数 */
    uint32_t timeout_count;        /* 超时次数 */
    uint32_t crc_error_count;      /* CRC错误次数 */
    uint32_t exception_count;      /* 异常响应次数 */
} ModbusStatus_t;

#define MODBUS_MAX_SLAVE_ID 247

/* 公共函数 */
int modbus_create_mutex(void);
void modbus_create_rx_semaphore(void);
uint16_t modbus_crc16(uint8_t *buffer, uint16_t len);


/* 状态函数 */
ModbusStatus_t modbus_get_status(void);
void modbus_reset_status(void);
uint32_t modbus_get_slave_error_count(uint8_t slave);
void modbus_reset_all_slave_errors(void);

/* 错误日志 */
#define MODBUS_ERR_LOG_SIZE 32

typedef struct {
    uint32_t tick;
    uint8_t  slave;
    uint8_t  func;
    int8_t   error_code;
    uint8_t  exception_code;
    uint16_t detail;
} ModbusErrLogEntry_t;

void modbus_print_error_log(void);
void modbus_clear_error_log(void);
uint32_t modbus_get_error_log_count(void);

/* Modbus API函数 */
int modbus_read_holding_registers(uint8_t slave, uint16_t start_addr, uint16_t count, uint16_t *dest);
int modbus_read_input_registers(uint8_t slave, uint16_t start_addr, uint16_t count, uint16_t *dest);
int modbus_write_single_register(uint8_t slave, uint16_t addr, uint16_t value);
int modbus_write_multiple_registers(uint8_t slave, uint16_t start_addr, uint16_t count, uint16_t *values);
int modbus_read_coils(uint8_t slave, uint16_t start_addr, uint16_t count, uint8_t *dest);
int modbus_write_single_coil(uint8_t slave, uint16_t addr, uint8_t state);


#endif
