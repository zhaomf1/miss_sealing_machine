#ifndef __MODBUS_SLAVE_REG_H
#define __MODBUS_SLAVE_REG_H

#include <stdint.h>
#include "cmsis_os2.h"

/* ===================================================================
 *        Modbus 从机寄存器业务层 — 公开接口
 *
 *  封装寄存器映射表、读写分发、动作状态追踪等所有业务逻辑。
 *  usart_comm.c 只负责 Modbus 协议组帧/解帧，
 *  通过调用这里的接口完成实际寄存器操作。
 * =================================================================== */

/* 寄存器访问属性 */
#define REG_ACCESS_RO  1   // 只读
#define REG_ACCESS_WO  2   // 只写
#define REG_ACCESS_RW  3   // 可读可写

/* 寄存器数据类型 */
#define REG_TYPE_U16   0   // 16 位寄存器
#define REG_TYPE_U32   1   // 32 位寄存器（占用连续 2 个地址）

/* 系统工作状态 (寄存器 0x0013) */
#define SYS_STATE_UNINITIALIZED   0x0000   // 未初始化
#define SYS_STATE_INITIALIZING    0x0001   // 初始化中
#define SYS_STATE_INITIALIZED     0x0002   // 初始化完成，待命
#define SYS_STATE_RUNNING         0x0003   // 流程执行中
#define SYS_STATE_FINISHED        0x0004   // 流程结束
#define SYS_STATE_WORKFLOW_FAILED 0x0005   // 流程执行失败

/* 写操作内部错误码 */
#define WRITE_OK                  0
#define WRITE_ERR_ILLEGAL        -1
#define WRITE_ERR_QUEUE_FULL     -2
#define WRITE_ERR_DEVICE         -3

/* 外设阻塞等待被结束流程命令取消 */
#define ACTION_WAIT_CANCELLED     -4

/* 寄存器描述符 */
typedef struct {
    uint16_t addr;          // 寄存器地址
    uint8_t  access;        // 访问属性 (REG_ACCESS_*)
    uint8_t  type;          // 数据类型 (REG_TYPE_*)
} ModbusRegDef_t;

/**
 * @brief 在寄存器映射表中二分查找指定地址的寄存器定义
 * @param addr 寄存器地址
 * @return 找到返回描述符指针，未找到返回 NULL
 */
const ModbusRegDef_t *modbus_reg_lookup(uint16_t addr);

/**
 * @brief 检查寄存器是否存在且具备要求的访问权限
 * @param addr        寄存器地址
 * @param need_access 要求的访问权限 (1=可读, 2=可写)
 * @return 1=满足, 0=不满足
 *
 * 自动处理 uint32 寄存器的高半地址（如 0x0071 属于 0x0070）。
 */
int modbus_reg_check_access(uint16_t addr, uint8_t need_access);

/**
 * @brief 读取指定寄存器地址的当前值（16 位）
 * @param addr      寄存器地址
 * @param out_value 输出：读取到的值
 * @return 0=成功, -1=寄存器不可读
 *
 * 根据寄存器类型自动分发到对应数据源：
 *   系统信息（版本/地址/故障）→ 内存变量
 *   实时温度 (0x0020 读)    → temp_ctrl_get()
 *   温控开关 (0x0021 读)    → temp_ctrl_is_running()
 *   调试动作状态 (0x00A0)    → 动作状态追踪变量
 *   EEPROM 参数 (0x0070~)   → AT24C02 存储
 *   其他                    → 影子缓存
 */
int modbus_reg_read_value(uint16_t addr, uint16_t *out_value);

/**
 * @brief 执行 16 位寄存器写操作（Modbus 功能码 0x06）
 * @param addr  寄存器地址
 * @param value 写入值
 * @return 0=成功, -1=寄存器不可写或参数无效
 *
 * 对于调试动作寄存器（0x0040/0x0041/0x0050），
 * 会同步记录执行结果到 0x00A0 和故障位图 0x0002。
 */
int modbus_reg_write_execute(uint16_t addr, uint16_t value);

/**
 * @brief 执行 32 位寄存器写操作（Modbus 功能码 0x10）
 * @param start_addr 起始寄存器地址
 * @param value      32 位写入值
 * @return 0=成功, -1=失败
 */
int modbus_reg_write32_execute(uint16_t start_addr, uint32_t value);

/**
 * @brief 获取当前 Modbus 从机地址
 * @return 从机地址 (1~247)
 */
uint8_t modbus_get_slave_addr(void);

/**
 * @brief 应用待生效的新从机地址
 *
 * 修改地址命令的响应帧必须使用旧地址，因此业务层先暂存新地址，
 * 待协议层发送完响应后再调用本接口使其生效。
 */
void modbus_apply_pending_slave_addr(void);

/** 查询当前是否收到结束流程请求，供外设等待循环及时退出 */
uint8_t modbus_reg_is_stop_requested(void);

/** 更新温度故障位 (0x0002 bit0)，fault=1异常，fault=0正常 */
void modbus_reg_set_temp_fault(uint8_t fault);

/**
 * @brief 上电初始化 EEPROM 参数缓存
 *
 * 从 AT24C02 一次性读取所有 EEPROM 参数到全局缓存，
 * 并同步更新寄存器影子缓存。
 * 应在 I2C 初始化完成后、首次 Modbus 通讯前调用。
 */
void modbus_reg_eeprom_cache_init(void);

/* ---- 复合动作异步执行队列 & 任务 (MX_FREERTOS_Init 中创建) ---- */

typedef struct {
    uint8_t  reg_addr;   // 寄存器地址
    uint16_t value;      // 16位参数
    int32_t  value32;    // 32位参数 (0x0030)
} CompoundActionMsg_t;

/** 复合动作消息队列 — 由 freertos.c 初始化 */
extern osMessageQueueId_t compound_action_queue;

/** 复合动作后台任务入口 — 由 freertos.c 创建 */
void compound_action_task(void *argument);

/**
 * @brief 获取当前系统工作状态（寄存器 0x0012）
 */
uint16_t modbus_reg_get_system_state(void);

/**
 * @brief 设置系统工作状态
 */
void modbus_reg_set_system_state(uint16_t state);


#endif
