#include "modbus_rtu.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>

/* 默认配置 */
static ModbusConfig_t modbus_config = {
    .timeout_ms = 200,      //修改等待时间
    .retry_count = 5,       //重传次数
};

/* 状态统计 */
static ModbusStatus_t modbus_status = {
    .tx_count = 0,
    .rx_count = 0,
    .error_count = 0,
    .timeout_count = 0,
    .crc_error_count = 0,
    .exception_count = 0
};

static osMutexId_t modbus_mutex = NULL;
osSemaphoreId_t modbus_rx_sem = NULL; // 串口3接收信号量

static uint32_t slave_error_count[MODBUS_MAX_SLAVE_ID + 1];

static ModbusErrLogEntry_t err_log[MODBUS_ERR_LOG_SIZE];
static uint8_t  err_log_index  = 0;
static uint32_t err_log_total  = 0;

static const char* modbus_fc_name(uint8_t func)
{
    switch (func) {
        case MODBUS_FC_READ_COILS:           return "READ_COILS(01)";
        case MODBUS_FC_READ_DISCRETE_INPUTS: return "READ_DISCRETE(02)";
        case MODBUS_FC_READ_HOLDING_REGS:    return "READ_HOLDING(03)";
        case MODBUS_FC_READ_INPUT_REGS:      return "READ_INPUT(04)";
        case MODBUS_FC_WRITE_SINGLE_COIL:    return "WRITE_COIL(05)";
        case MODBUS_FC_WRITE_SINGLE_REG:     return "WRITE_REG(06)";
        case MODBUS_FC_WRITE_MULTIPLE_REGS:  return "WRITE_MULTI(10)";
        default: return "UNKNOWN_FC";
    }
}

static const char* modbus_exception_name(uint8_t code)
{
    switch (code) {
        case MODBUS_EXCEPTION_ILLEGAL_FUNCTION:      return "ILLEGAL_FUNC";
        case MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS:  return "ILLEGAL_ADDR";
        case MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE:    return "ILLEGAL_VAL";
        case MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE:  return "SLAVE_FAILURE";
        case MODBUS_EXCEPTION_ACKNOWLEDGE:           return "ACK";
        case MODBUS_EXCEPTION_SLAVE_BUSY:            return "SLAVE_BUSY";
        case MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE:  return "NACK";
        case MODBUS_EXCEPTION_MEMORY_PARITY_ERROR:   return "MEM_PARITY";
        default: return "UNKNOWN_EXC";
    }
}

static void modbus_hex_dump(const uint8_t *data, uint16_t len)
{
    printf("       RX_RAW[%d]:", len);
    for (uint16_t i = 0; i < len && i < 64; i++) {
        printf(" %02X", data[i]);
    }
    printf("\r\n");
}

static void modbus_log_error(uint8_t slave, uint8_t func, int err, uint8_t exc, uint16_t detail)
{
    ModbusErrLogEntry_t *e = &err_log[err_log_index];

    e->tick           = osKernelGetTickCount();
    e->slave          = slave;
    e->func           = func;
    e->error_code     = (int8_t)err;
    e->exception_code = exc;
    e->detail         = detail;

    err_log_index = (err_log_index + 1) % MODBUS_ERR_LOG_SIZE;
    err_log_total++;

    printf("[MODBUS_ERR] slave=0x%02X func=%s", slave, modbus_fc_name(func));

    switch (err) {
        case MODBUS_ERR_MUTEX:
            printf(" MUTEX timeout\r\n");
            break;
        case MODBUS_ERR_SEND:
            printf(" SEND failed\r\n");
            break;
        case MODBUS_ERR_TIMEOUT:
            printf(" TIMEOUT no response\r\n");
            break;
        case MODBUS_ERR_CRC:
            printf(" CRC recv=0x%04X\r\n", detail);
            if (modbus_rx_len > 0) modbus_hex_dump(modbus_rtu_rx_backup, modbus_rx_len);
            break;
        case MODBUS_ERR_ADDR:
            printf(" ADDR expected=0x%02X recv=0x%02X\r\n", slave, (uint8_t)detail);
            if (modbus_rx_len > 0) modbus_hex_dump(modbus_rtu_rx_backup, modbus_rx_len);
            break;
        case MODBUS_ERR_EXCEPTION:
            printf(" EXCEPTION 0x%02X(%s)\r\n", exc, modbus_exception_name(exc));
            if (modbus_rx_len > 0) modbus_hex_dump(modbus_rtu_rx_backup, modbus_rx_len);
            break;
        case MODBUS_ERR_RESP_LEN:
            printf(" RESP_LEN rx_len=%d\r\n", detail);
            if (modbus_rx_len > 0) modbus_hex_dump(modbus_rtu_rx_backup, modbus_rx_len);
            break;
        case MODBUS_ERR_SEM:
            printf(" SEM null\r\n");
            break;
        default:
            printf(" UNKNOWN(%d)\r\n", err);
            break;
    }
}

void modbus_print_error_log(void)
{
    uint32_t count = (err_log_total > MODBUS_ERR_LOG_SIZE) ? MODBUS_ERR_LOG_SIZE : err_log_total;
    uint8_t  start = (err_log_total > MODBUS_ERR_LOG_SIZE) ? err_log_index : 0;

    printf("=== Modbus Error Log (%lu total, showing %lu) ===\r\n", err_log_total, count);

    for (uint32_t i = 0; i < count; i++) {
        uint8_t idx = (start + i) % MODBUS_ERR_LOG_SIZE;
        ModbusErrLogEntry_t *e = &err_log[idx];

        printf("#%lu tick=%lu slave=0x%02X func=%s err=",
               i + 1, e->tick, e->slave, modbus_fc_name(e->func));

        switch (e->error_code) {
            case MODBUS_ERR_MUTEX:     printf("MUTEX"); break;
            case MODBUS_ERR_SEND:      printf("SEND"); break;
            case MODBUS_ERR_TIMEOUT:   printf("TIMEOUT"); break;
            case MODBUS_ERR_CRC:       printf("CRC(0x%04X)", e->detail); break;
            case MODBUS_ERR_ADDR:      printf("ADDR(recv=0x%02X)", (uint8_t)e->detail); break;
            case MODBUS_ERR_EXCEPTION: printf("EXC(0x%02X_%s)", e->exception_code, modbus_exception_name(e->exception_code)); break;
            case MODBUS_ERR_RESP_LEN:  printf("LEN(%d)", e->detail); break;
            case MODBUS_ERR_SEM:       printf("SEM"); break;
            default: printf("%d", e->error_code); break;
        }
        printf("\r\n");
    }
}

void modbus_clear_error_log(void)
{
    memset(err_log, 0, sizeof(err_log));
    err_log_index = 0;
    err_log_total = 0;
}

uint32_t modbus_get_error_log_count(void)
{
    return err_log_total;
}


static const osMutexAttr_t modbus_mutex_attr = {
    .name      = "ModbusMutex",        
    .attr_bits = osMutexPrioInherit,    
    .cb_mem    = NULL,                 
    .cb_size   = 0
};

/* CRC16 查表 */
static const uint16_t crc_tab[256] = {
    0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,0xC601,0x06C0,0x0780,0xC741,0x0500,0xC5C1,0xC481,0x0440,
    0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,0xCFC1,0xCE81,0x0E40,0x0A00,0xCAC1,0xCB81,0x0B40,0xC901,0x09C0,0x0880,0xC841,
    0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,0x1E00,0xDEC1,0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,
    0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,
    0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,0xF281,0x3240,0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,
    0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,
    0x2800,0xE8C1,0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,
    0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,
    0xA001,0x60C0,0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,0x6600,0xA6C1,0xA781,0x6740,0xA501,0x65C0,0x6480,0xA441,
    0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,0xAA01,0x6AC0,0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,
    0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,
    0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,0xB681,0x7640,0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,0x7080,0xB041,
    0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,0x5280,0x9241,0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,
    0x9C01,0x5CC0,0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,0x5A00,0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,
    0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,0x4A40,0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,
    0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,0x8201,0x42C0,0x4380,0x8341,0x4100,0x81C1,0x8081,0x4040
};


uint16_t modbus_crc16(uint8_t *buffer, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ crc_tab[(crc ^ buffer[i]) & 0xFF];
    }
    return crc;
}

/* 创建互斥锁 */
int modbus_create_mutex(void)
{
    modbus_mutex = osMutexNew(&modbus_mutex_attr);
    if (modbus_mutex == NULL) {
        return MODBUS_ERR_MUTEX;
    }
    return MODBUS_OK;
}

/**
 * @brief 创建接收信号量
 */
void modbus_create_rx_semaphore(void)
{
    // 创建二值信号量，初始值为 0
    modbus_rx_sem = osSemaphoreNew(1, 0, NULL);
}

/* 内部事务处理函数 */
static int modbus_transaction(uint8_t slave, uint8_t func, uint8_t *req, uint16_t req_len,
                              uint8_t *resp, uint16_t *resp_len, uint8_t *exception_code)
{
    uint8_t tx_buf[256];
    uint16_t crc;

    // 获取互斥锁（超时时间 = 2倍最坏事务时间，确保不会误超时）
    uint32_t mutex_timeout = (uint32_t)(modbus_config.retry_count + 2) * 2 * modbus_config.timeout_ms;
    if (!modbus_mutex || osMutexAcquire(modbus_mutex, mutex_timeout) != osOK) {
        modbus_status.error_count++;
        if (slave <= MODBUS_MAX_SLAVE_ID) slave_error_count[slave]++;
        modbus_log_error(slave, func, MODBUS_ERR_MUTEX, 0, 0);
        return MODBUS_ERR_MUTEX;
    }

    // 获取信号量
    if (modbus_rx_sem == NULL) {
        osMutexRelease(modbus_mutex);
        modbus_status.error_count++;
        if (slave <= MODBUS_MAX_SLAVE_ID) slave_error_count[slave]++;
        modbus_log_error(slave, func, MODBUS_ERR_SEM, 0, 0);
        return MODBUS_ERR_SEM;
    }

    // 2. 组包发送
    tx_buf[0] = slave;
    tx_buf[1] = func;
    memcpy(&tx_buf[2], req, req_len);
    crc = modbus_crc16(tx_buf, req_len + 2);
    tx_buf[req_len + 2] = crc & 0xFF;
    tx_buf[req_len + 3] = (crc >> 8) & 0xFF;

    modbus_rs485_rx_begin();

    // 发送数据
    if (rs485_transmit(tx_buf, req_len + 4, modbus_config.timeout_ms) != 0) {
        osMutexRelease(modbus_mutex);
        modbus_status.error_count++;
        if (slave <= MODBUS_MAX_SLAVE_ID) slave_error_count[slave]++;
        modbus_log_error(slave, func, MODBUS_ERR_SEND, 0, 0);
        return MODBUS_ERR_SEND;
    }

    modbus_status.tx_count++;

    // 3. 等待接收完成 (使用信号量挂起任务，释放 CPU)
    if (osSemaphoreAcquire(modbus_rx_sem, modbus_config.timeout_ms) != osOK) {
        osMutexRelease(modbus_mutex);
        modbus_status.timeout_count++;
        modbus_status.error_count++;
        if (slave <= MODBUS_MAX_SLAVE_ID) slave_error_count[slave]++;
        modbus_log_error(slave, func, MODBUS_ERR_TIMEOUT, 0, 0);
        return MODBUS_ERR_TIMEOUT;
    }

    modbus_rx_len = modbus_rs485_rx_end(10);

    // 4. 数据校验与解析

    // 基本长度检查：地址(1) + 功能码(1) + CRC(2) = 4字节
    if (modbus_rx_len < 4) {
        osMutexRelease(modbus_mutex);
        modbus_status.error_count++;
        if (slave <= MODBUS_MAX_SLAVE_ID) slave_error_count[slave]++;
        modbus_log_error(slave, func, MODBUS_ERR_RESP_LEN, 0, modbus_rx_len);
        return MODBUS_ERR_RESP_LEN;
    }

    // 校验地址
    if (modbus_rtu_rx_backup[0] != slave) {
        osMutexRelease(modbus_mutex);
        modbus_status.error_count++;
        if (slave <= MODBUS_MAX_SLAVE_ID) slave_error_count[slave]++;
        modbus_log_error(slave, func, MODBUS_ERR_ADDR, 0, modbus_rtu_rx_backup[0]);
        return MODBUS_ERR_ADDR;
    }

    // 校验 CRC
    uint16_t recv_crc = (modbus_rtu_rx_backup[modbus_rx_len-1] << 8) | modbus_rtu_rx_backup[modbus_rx_len-2];
    uint16_t calc_crc = modbus_crc16(modbus_rtu_rx_backup, modbus_rx_len - 2);
    if (recv_crc != calc_crc) {
        osMutexRelease(modbus_mutex);
        modbus_status.crc_error_count++;
        modbus_status.error_count++;
        if (slave <= MODBUS_MAX_SLAVE_ID) slave_error_count[slave]++;
        modbus_log_error(slave, func, MODBUS_ERR_CRC, 0, recv_crc);
        return MODBUS_ERR_CRC;
    }

    // 检查异常响应
    if (modbus_rtu_rx_backup[1] & 0x80) {
        uint8_t exc = 0;
        if (exception_code && modbus_rx_len >= 3) {
            exc = modbus_rtu_rx_backup[2];
            *exception_code = exc;
        }
        osMutexRelease(modbus_mutex);
        modbus_status.exception_count++;
        modbus_status.error_count++;
        if (slave <= MODBUS_MAX_SLAVE_ID) slave_error_count[slave]++;
        modbus_log_error(slave, func, MODBUS_ERR_EXCEPTION, exc, 0);
        return MODBUS_ERR_EXCEPTION;
    }

    modbus_status.rx_count++;

    // 5. 提取有效数据
    uint8_t *data_ptr = &modbus_rtu_rx_backup[2]; // 默认指向数据区起始位置
    uint16_t data_len = modbus_rx_len - 4;         // 默认数据长度 = 总长 - 地址 - 功能码 - CRC

    // 对于读操作，第3个字节是 Byte Count，需要跳过它
    if (func == MODBUS_FC_READ_HOLDING_REGS || func == MODBUS_FC_READ_INPUT_REGS ||
        func == MODBUS_FC_READ_COILS || func == MODBUS_FC_READ_DISCRETE_INPUTS) {
        if (modbus_rx_len < 3) {
            osMutexRelease(modbus_mutex);
            modbus_status.error_count++;
            if (slave <= MODBUS_MAX_SLAVE_ID) slave_error_count[slave]++;
             modbus_log_error(slave, func, MODBUS_ERR_RESP_LEN, 0, modbus_rx_len);
            return MODBUS_ERR_RESP_LEN;
        }
        data_ptr = &modbus_rtu_rx_backup[3]; // 跳过 Byte Count
        data_len = modbus_rtu_rx_backup[2];  // 数据长度等于 Byte Count

        // Byte Count 合理性校验：防止帧间 IDLE 误触发导致半帧
        if (3 + data_len + 2 > modbus_rx_len) {
            osMutexRelease(modbus_mutex);
            modbus_status.error_count++;
            if (slave <= MODBUS_MAX_SLAVE_ID) slave_error_count[slave]++;
            modbus_log_error(slave, func, MODBUS_ERR_RESP_LEN, 0, modbus_rx_len);
            return MODBUS_ERR_RESP_LEN;
        }
    }

    // 拷贝数据到用户缓冲区
    if (resp && data_len > 0) {
        // 对于寄存器，进行大小端转换
        if (func == MODBUS_FC_READ_HOLDING_REGS || func == MODBUS_FC_READ_INPUT_REGS) {
            for (uint16_t i = 0; i < data_len / 2; i++) {
                ((uint16_t*)resp)[i] = (data_ptr[i*2] << 8) | data_ptr[i*2+1];
            }
        } else {
            // 线圈或离散量直接拷贝
            memcpy(resp, data_ptr, data_len);
        }
    }

    if (resp_len) {
        *resp_len = data_len;
    }

    osMutexRelease(modbus_mutex);
    return MODBUS_OK;
}


/***************************************************************************************
 * 状态管理函数 
 ***************************************************************************************/

ModbusStatus_t modbus_get_status(void)
{
    return modbus_status;
}

void modbus_reset_status(void)
{
    memset(&modbus_status, 0, sizeof(ModbusStatus_t));
}

uint32_t modbus_get_slave_error_count(uint8_t slave)
{
    if (slave > MODBUS_MAX_SLAVE_ID) return 0;
    return slave_error_count[slave];
}

void modbus_reset_all_slave_errors(void)
{
    memset(slave_error_count, 0, sizeof(slave_error_count));
}


/***************************************************************************************
 * 公共API 
 ***************************************************************************************/

/**
 * @brief  读取保持寄存器 (Modbus 功能码 0x03)
 * @note   用于读取从站设备的可读写参数或模拟量输出。
 * 
 * @param  slave: 从站地址 (1-247)
 * @param  start_addr: 起始寄存器地址
 * @param  count: 要读取的寄存器数量
 * @param  dest: 指向存储读取数据的缓冲区指针 (数据以 uint16_t 数组形式存储)
 * @retval int: 操作状态码
 *           - MODBUS_OK: 读取成功
 *           - MODBUS_ERR_...: 详见错误码定义 (超时、CRC错误、异常响应等)
 */
int modbus_read_holding_registers(uint8_t slave, uint16_t start_addr, uint16_t count, uint16_t *dest)
{
    // 参数验证
    if (slave < 1 || slave > 247) {
        return MODBUS_ERR_PARM;
    }
    if (count < 1 || count > 125) {
        return MODBUS_ERR_PARM;
    }
    if (!dest) {
        return MODBUS_ERR_PARM;
    }
    
    uint8_t req[4];
    req[0] = start_addr >> 8;
    req[1] = start_addr & 0xFF;
    req[2] = count >> 8;
    req[3] = count & 0xFF;
    
    uint8_t exception_code = 0;
    int ret;
    
    // 实现重试机制
    for (uint8_t i = 0; i <= modbus_config.retry_count; i++) {
        ret = modbus_transaction(slave, MODBUS_FC_READ_HOLDING_REGS, req, 4, (uint8_t*)dest, NULL, &exception_code);
        if (ret == MODBUS_OK) {
            if (i > 0) printf("[MODBUS_OK] slave=0x%02X recovered retry=%d\r\n", slave, i);
            return ret;
        }
        if (i < modbus_config.retry_count) {
            osDelay(10);
        }
    }
    
    printf("[MODBUS_FAIL] slave=0x%02X func=READ_HOLDING all retries exhausted err=%d\r\n", slave, ret);
    return ret;
}

/**
 * @brief  读取输入寄存器 (Modbus 功能码 0x04)
 * @note   用于读取从站设备的只读数据，如传感器采集的实时数值。
 * 
 * @param  slave: 从站地址 (1-247)
 * @param  start_addr: 起始寄存器地址
 * @param  count: 要读取的寄存器数量
 * @param  dest: 指向存储读取数据的缓冲区指针 (数据以 uint16_t 数组形式存储)
 * @retval int: 操作状态码
 *           - MODBUS_OK: 读取成功
 *           - MODBUS_ERR_...: 详见错误码定义
 */
int modbus_read_input_registers(uint8_t slave, uint16_t start_addr, uint16_t count, uint16_t *dest)
{
    // 参数验证
    if (slave < 1 || slave > 247) {
        return MODBUS_ERR_PARM;
    }
    if (count < 1 || count > 125) {
        return MODBUS_ERR_PARM;
    }
    if (!dest) {
        return MODBUS_ERR_PARM;
    }
    
    uint8_t req[4];
    req[0] = start_addr >> 8;
    req[1] = start_addr & 0xFF;
    req[2] = count >> 8;
    req[3] = count & 0xFF;
    
    uint8_t exception_code = 0;
    int ret;
    
    // 实现重试机制
    for (uint8_t i = 0; i <= modbus_config.retry_count; i++) {
        ret = modbus_transaction(slave, MODBUS_FC_READ_INPUT_REGS, req, 4, (uint8_t*)dest, NULL, &exception_code);
        if (ret == MODBUS_OK) {
            if (i > 0) printf("[MODBUS_OK] slave=0x%02X recovered retry=%d\r\n", slave, i);
            return ret;
        }
        if (i < modbus_config.retry_count) {
            osDelay(10);
        }
    }
    
    printf("[MODBUS_FAIL] slave=0x%02X func=READ_INPUT all retries exhausted err=%d\r\n", slave, ret);
    return ret;
}

/**
 * @brief  写单个保持寄存器 (Modbus 功能码 0x06)
 * @note   用于设置从站设备的某个具体参数。
 * 
 * @param  slave: 从站地址 (1-247)
 * @param  addr: 寄存器地址
 * @param  value: 要写入的数值
 * @retval int: 操作状态码
 *           - MODBUS_OK: 写入成功
 *           - MODBUS_ERR_...: 详见错误码定义
 */
int modbus_write_single_register(uint8_t slave, uint16_t addr, uint16_t value)
{
    // 参数验证
    if (slave < 1 || slave > 247) {
        return MODBUS_ERR_PARM;
    }
    
    uint8_t req[4];
    req[0] = addr >> 8;
    req[1] = addr & 0xFF;
    req[2] = value >> 8;
    req[3] = value & 0xFF;
    
    uint8_t exception_code = 0;
    int ret;
    
    // 实现重试机制
    for (uint8_t i = 0; i <= modbus_config.retry_count; i++) {
        ret = modbus_transaction(slave, MODBUS_FC_WRITE_SINGLE_REG, req, 4, NULL, NULL, &exception_code);
        if (ret == MODBUS_OK) {
            if (i > 0) printf("[MODBUS_OK] slave=0x%02X recovered retry=%d\r\n", slave, i);
            return ret;
        }
        if (i < modbus_config.retry_count) {
            osDelay(10);
        }
    }
    
    printf("[MODBUS_FAIL] slave=0x%02X func=WRITE_REG all retries exhausted err=%d\r\n", slave, ret);
    return ret;
}

/**
 * @brief  写多个保持寄存器 (Modbus 功能码 0x10)
 * @note   用于一次性写入多个连续的寄存器，效率较高。
 * 
 * @param  slave: 从站地址 (1-247)
 * @param  start_addr: 起始寄存器地址
 * @param  count: 要写入的寄存器数量
 * @param  values: 指向包含写入数据的 uint16_t 数组指针
 * @retval int: 操作状态码
 *           - MODBUS_OK: 写入成功
 *           - MODBUS_ERR_...: 详见错误码定义
 */
int modbus_write_multiple_registers(uint8_t slave, uint16_t start_addr, uint16_t count, uint16_t *values)
{
    // 参数验证
    if (slave < 1 || slave > 247) {
        return MODBUS_ERR_PARM;
    }
    if (count < 1 || count > 120) {
        return MODBUS_ERR_PARM;
    }
    if (!values) {
        return MODBUS_ERR_PARM;
    }
    
    uint8_t req[256]; 
    req[0] = start_addr >> 8;
    req[1] = start_addr & 0xFF;
    req[2] = count >> 8;
    req[3] = count & 0xFF;
    req[4] = count * 2;
    for (uint16_t i = 0; i < count; i++) {
        req[5 + i*2] = values[i] >> 8;
        req[5 + i*2 + 1] = values[i] & 0xFF;
    }
    
    uint8_t exception_code = 0;
    int ret;
    
    // 实现重试机制
    for (uint8_t i = 0; i <= modbus_config.retry_count; i++) {
        ret = modbus_transaction(slave, MODBUS_FC_WRITE_MULTIPLE_REGS, req, 5 + count*2, NULL, NULL, &exception_code);
        if (ret == MODBUS_OK) {
            if (i > 0) printf("[MODBUS_OK] slave=0x%02X recovered retry=%d\r\n", slave, i);
            return ret;
        }
        if (i < modbus_config.retry_count) {
            osDelay(10);
        }
    }
    
    printf("[MODBUS_FAIL] slave=0x%02X func=WRITE_MULTI all retries exhausted err=%d\r\n", slave, ret);
    return ret;
}

/**
 * @brief  读取线圈 (Modbus 功能码 0x01)
 * @note   用于读取从站的开关量输出状态。
 *         该函数会将 Modbus 返回的打包字节流解包为独立的 uint8_t 数组 (0 或 1)。
 * 
 * @param  slave: 从站地址 (1-247)
 * @param  start_addr: 起始线圈地址
 * @param  count: 要读取的线圈数量
 * @param  dest: 指向存储读取状态的缓冲区指针 (每个元素代表一个线圈，0=OFF, 1=ON)
 * @retval int: 操作状态码
 *           - MODBUS_OK: 读取成功
 *           - MODBUS_ERR_...: 详见错误码定义
 */
int modbus_read_coils(uint8_t slave, uint16_t start_addr, uint16_t count, uint8_t *dest)
{
    // 参数验证
    if (slave < 1 || slave > 247) {
        return MODBUS_ERR_PARM;
    }
    if (count < 1 || count > 2000) {
        return MODBUS_ERR_PARM;
    }
    if (!dest) {
        return MODBUS_ERR_PARM;
    }
    
    uint8_t req[4];
    req[0] = start_addr >> 8;
    req[1] = start_addr & 0xFF;
    req[2] = count >> 8;
    req[3] = count & 0xFF;
    
    uint8_t data[256]; // 2000 bits = 250 bytes
    uint8_t exception_code = 0;
    int ret;
    
    // 实现重试机制
    for (uint8_t i = 0; i <= modbus_config.retry_count; i++) {
        ret = modbus_transaction(slave, MODBUS_FC_READ_COILS, req, 4, data, NULL, &exception_code);
        if (ret == MODBUS_OK) {
            if (i > 0) printf("[MODBUS_OK] slave=0x%02X recovered retry=%d\r\n", slave, i);
            for (uint16_t i = 0; i < count; i++) {
                dest[i] = (data[i/8] >> (i%8)) & 1;
            }
            return ret;
        }
        if (i < modbus_config.retry_count) {
            osDelay(10);
        }
    }
    
    printf("[MODBUS_FAIL] slave=0x%02X func=READ_COILS all retries exhausted err=%d\r\n", slave, ret);
    return ret;
}

/**
 * @brief  写单个线圈 (Modbus 功能码 0x05)
 * @note   用于控制从站的某个开关量输出。
 *         协议规定：0xFF00 代表 ON (闭合)，0x0000 代表 OFF (断开)。
 * 
 * @param  slave: 从站地址 (1-247)
 * @param  addr: 线圈地址
 * @param  state: 状态 (非 0 为 ON，0 为 OFF)
 * @retval int: 操作状态码
 *           - MODBUS_OK: 写入成功
 *           - MODBUS_ERR_...: 详见错误码定义
 */
int modbus_write_single_coil(uint8_t slave, uint16_t addr, uint8_t state)
{
    // 参数验证
    if (slave < 1 || slave > 247) {
        return MODBUS_ERR_PARM;
    }
    
    uint8_t req[4];
    req[0] = addr >> 8;
    req[1] = addr & 0xFF;
    req[2] = state ? 0xFF : 0x00;
    req[3] = 0x00;
    
    uint8_t exception_code = 0;
    int ret;
    
    // 实现重试机制
    for (uint8_t i = 0; i <= modbus_config.retry_count; i++) {
        ret = modbus_transaction(slave, MODBUS_FC_WRITE_SINGLE_COIL, req, 4, NULL, NULL, &exception_code);
        if (ret == MODBUS_OK) {
            if (i > 0) printf("[MODBUS_OK] slave=0x%02X recovered retry=%d\r\n", slave, i);
            return ret;
        }
        if (i < modbus_config.retry_count) {
            osDelay(10);
        }
    }
    
    printf("[MODBUS_FAIL] slave=0x%02X func=WRITE_COIL all retries exhausted err=%d\r\n", slave, ret);
    return ret;
}
