#include "stdio.h"
#include "string.h"
#include "usart_comm.h"
#include "usart.h"
#include "modbus_slave_reg.h"
#include "modbus_rtu.h"


/* ===================================================================
 *        Modbus RTU Slave — 协议组帧/解帧层（纯传输，无业务逻辑）
 *
 *  所有寄存器读写操作通过 app_control.h 的公开接口完成：
 *    modbus_reg_check_access()     — 检查寄存器是否存在及权限
 *    modbus_reg_read_value()       — 读取寄存器值
 *    modbus_reg_write_execute()    — 执行 16 位写操作
 *    modbus_reg_write32_execute()  — 执行 32 位写操作
 *    modbus_get_slave_addr()       — 获取当前从机地址
 *    modbus_reg_lookup()           — 查找寄存器定义
 * =================================================================== */

#define MODBUS_TX_BUF_SIZE  256

static uint8_t modbus_tx_buf[MODBUS_TX_BUF_SIZE];

/* ===================================================================
 *                    CRC / 传输 底层
 * =================================================================== */

/**
 * @brief 计算 Modbus CRC16
 */
static uint16_t modbus_build_crc(uint8_t *buf, uint16_t len)
{
    return modbus_crc16(buf, len);
}

/**
 * @brief 发送 Modbus 响应帧
 */
static void modbus_send_response(uint8_t *buf, uint16_t len)
{
    host_transmit(buf, len);
}

/**
 * @brief 发送 Modbus 异常响应
 */
static void modbus_send_exception(uint8_t func, uint8_t code)
{
    uint8_t buf[5];
    buf[0] = modbus_get_slave_addr();
    buf[1] = func | 0x80;
    buf[2] = code;
    uint16_t crc = modbus_build_crc(buf, 3);
    buf[3] = crc & 0xFF;
    buf[4] = (crc >> 8) & 0xFF;
    modbus_send_response(buf, 5);
}

/* ===================================================================
 *                 Modbus 功能码处理
 * =================================================================== */

/**
 * @brief 处理功能码 0x03 — 读保持寄存器
 *
 * 逐地址检查可读性 → 逐地址读取值 → 组帧发送。
 */
static void modbus_handle_read_holding(uint8_t *rx, uint16_t rx_len)
{
    if (rx_len != 8) {
        modbus_send_exception(0x03, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    uint16_t start_addr = ((uint16_t)rx[2] << 8) | rx[3];
    uint16_t reg_count  = ((uint16_t)rx[4] << 8) | rx[5];

    if (reg_count == 0 || reg_count > 32) {
        modbus_send_exception(0x03, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    /* 校验全部地址是否可读 */
    for (uint16_t i = 0; i < reg_count; i++) {
        if (!modbus_reg_check_access(start_addr + i, 1)) {
            modbus_send_exception(0x03, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
            return;
        }
    }

    uint16_t byte_count = reg_count * 2;
    if (byte_count + 5 > MODBUS_TX_BUF_SIZE) {
        modbus_send_exception(0x03, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    modbus_tx_buf[0] = rx[0];
    modbus_tx_buf[1] = 0x03;
    modbus_tx_buf[2] = (uint8_t)byte_count;

    for (uint16_t i = 0; i < reg_count; i++) {
        uint16_t val = 0;
        if (modbus_reg_read_value(start_addr + i, &val) != 0) {
            modbus_send_exception(0x03, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
            return;
        }
        modbus_tx_buf[3 + i * 2]     = (uint8_t)((val >> 8) & 0xFF);
        modbus_tx_buf[3 + i * 2 + 1] = (uint8_t)(val & 0xFF);
    }

    uint16_t data_len = 3 + byte_count;
    uint16_t crc = modbus_build_crc(modbus_tx_buf, data_len);
    modbus_tx_buf[data_len]     = (uint8_t)(crc & 0xFF);
    modbus_tx_buf[data_len + 1] = (uint8_t)((crc >> 8) & 0xFF);

    modbus_send_response(modbus_tx_buf, data_len + 2);
}

/**
 * @brief 处理功能码 0x06 — 写单个寄存器
 *
 * 校验可写性 → 执行写操作 → 回显响应。
 */
static void modbus_handle_write_single(uint8_t *rx, uint16_t rx_len)
{
    if (rx_len != 8) {
        modbus_send_exception(0x06, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    uint16_t addr  = ((uint16_t)rx[2] << 8) | rx[3];
    uint16_t value = ((uint16_t)rx[4] << 8) | rx[5];

    if (!modbus_reg_check_access(addr, 2)) {
        modbus_send_exception(0x06, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }

    int write_ret = modbus_reg_write_execute(addr, value);
    if (write_ret == WRITE_ERR_QUEUE_FULL) {
        modbus_send_exception(0x06, MODBUS_EXCEPTION_SLAVE_BUSY);
        return;
    } else if (write_ret == WRITE_ERR_DEVICE) {
        modbus_send_exception(0x06, MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE);
        return;
    } else if (write_ret != WRITE_OK) {
        modbus_send_exception(0x06, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    /* 回显 */
    uint8_t resp[8];
    /* 修改地址时响应仍使用请求中的旧地址 */
    resp[0] = rx[0];
    resp[1] = rx[1];
    resp[2] = rx[2];
    resp[3] = rx[3];
    resp[4] = rx[4];
    resp[5] = rx[5];
    uint16_t crc = modbus_build_crc(resp, 6);
    resp[6] = crc & 0xFF;
    resp[7] = (crc >> 8) & 0xFF;
    modbus_send_response(resp, 8);
    if (addr == 0x0001) {
        modbus_apply_pending_slave_addr();
    }
}

/**
 * @brief 处理功能码 0x10 — 写多个寄存器
 *
 * 支持 uint16（1 个寄存器）和 uint32（2 个寄存器）两种写入。
 */
static void modbus_handle_write_multiple(uint8_t *rx, uint16_t rx_len)
{
    if (rx_len < 9) {
        modbus_send_exception(0x10, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    uint16_t start_addr = ((uint16_t)rx[2] << 8) | rx[3];
    uint16_t reg_count  = ((uint16_t)rx[4] << 8) | rx[5];
    uint8_t  byte_count = rx[6];

    if (reg_count == 0 || reg_count > 32 || byte_count != reg_count * 2) {
        modbus_send_exception(0x10, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    if (rx_len != (uint16_t)(9 + byte_count)) {
        modbus_send_exception(0x10, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    const ModbusRegDef_t *def = modbus_reg_lookup(start_addr);
    if (!def) {
        modbus_send_exception(0x10, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }

    if (def->type == REG_TYPE_U32 && reg_count == 2) {
        /* 32 位写入 */
        uint32_t val32 = ((uint32_t)rx[7] << 24) | ((uint32_t)rx[8] << 16)
                       | ((uint32_t)rx[9] << 8)  | rx[10];

        int ret32 = modbus_reg_write32_execute(start_addr, val32);
        if (ret32 == WRITE_ERR_QUEUE_FULL) {
            modbus_send_exception(0x10, MODBUS_EXCEPTION_SLAVE_BUSY);
            return;
        } else if (ret32 == WRITE_ERR_DEVICE) {
            modbus_send_exception(0x10, MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE);
            return;
        } else if (ret32 != 0) {
            modbus_send_exception(0x10, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
            return;
        }
    } else if (def->type == REG_TYPE_U16 && reg_count == 1) {
        /* 16 位写入 */
        uint16_t val16 = ((uint16_t)rx[7] << 8) | rx[8];

        int ret16 = modbus_reg_write_execute(start_addr, val16);
        if (ret16 == WRITE_ERR_QUEUE_FULL) {
            modbus_send_exception(0x10, MODBUS_EXCEPTION_SLAVE_BUSY);
            return;
        } else if (ret16 == WRITE_ERR_DEVICE) {
            modbus_send_exception(0x10, MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE);
            return;
        } else if (ret16 != WRITE_OK) {
            modbus_send_exception(0x10, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
            return;
        }
    } else {
        modbus_send_exception(0x10, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }

    /* 响应帧 */
    modbus_tx_buf[0] = rx[0];
    modbus_tx_buf[1] = 0x10;
    modbus_tx_buf[2] = rx[2];
    modbus_tx_buf[3] = rx[3];
    modbus_tx_buf[4] = rx[4];
    modbus_tx_buf[5] = rx[5];
    uint16_t crc = modbus_build_crc(modbus_tx_buf, 6);
    modbus_tx_buf[6] = (uint8_t)(crc & 0xFF);
    modbus_tx_buf[7] = (uint8_t)((crc >> 8) & 0xFF);

    modbus_send_response(modbus_tx_buf, 8);
    if (start_addr == 0x0001 && reg_count == 1) {
        modbus_apply_pending_slave_addr();
    }
}

/* ===================================================================
 *                Modbus Slave 帧入口 & FreeRTOS 任务
 * =================================================================== */

/**
 * @brief Modbus RTU 从机帧处理入口
 *
 * 校验地址/CRC → 按功能码分发。
 */
static void modbus_slave_handler(uint8_t *data, uint16_t len)
{
    if (len < 4) return;

    if (data[0] != modbus_get_slave_addr()) return;

    uint16_t recv_crc = ((uint16_t)data[len - 1] << 8) | data[len - 2];
    uint16_t calc_crc = modbus_build_crc(data, len - 2);
    if (recv_crc != calc_crc) return;

    uint8_t func = data[1];

    switch (func) {
        case 0x03:
            modbus_handle_read_holding(data, len);
            break;
        case 0x06:
            modbus_handle_write_single(data, len);
            break;
        case 0x10:
            modbus_handle_write_multiple(data, len);
            break;
        default:
            modbus_send_exception(func, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
            break;
    }
}

/**
 * @brief 与上位机 Modbus 通讯任务
 *
 * 从消息队列接收 USART1 收到的 Modbus 帧，交给协议处理函数。
 */
void uart_comm_task(void *argument)
{
    UartMsg_t rx_msg;

    for(;;)
    {
        if (osMessageQueueGet(uartRxQueueHandle, &rx_msg, NULL, osWaitForever) == osOK)
        {
            modbus_slave_handler(rx_msg.data, rx_msg.len);
        }
        osDelay(10);
    }
}
