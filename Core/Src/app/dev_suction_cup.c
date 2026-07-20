#include "dev_suction_cup.h"
#include "modbus_rtu.h"
#include "modbus_slave_reg.h"
#include "app_control.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdio.h>

#define SUCTION_POINT_BASE_ADDR     0x0100
#define SUCTION_POINT_REGS_PER      3

/**
 * @brief 吸盘上电初始化，配置IO模式和判断时间
 * @return 0=成功, -1=失败
 */
int dev_suction_cup_init(void)
{
    int ret = 0;
    /*关闭IO模式*/
    if (suction_cup_set_io_mode(0) != 0) {
        ret = -1;
    }
    osDelay(10);
    /*设置吸取判断时间为2秒*/
    if (suction_cup_set_judge_time(2) != 0) {
        ret = -1;
    }
    osDelay(10);
    /*设置最小真空度*/
    if (suction_cup_set_min_vacuum(15) != 0) {
        ret = -1;
    }
    osDelay(10);
    /*设置最大真空度*/
    if (suction_cup_set_max_vacuum(30) != 0) {
        ret = -1;
    }
    osDelay(10);
    /* 确保初始状态为释放 */
    suction_cup_release();  
    osDelay(10);

    return ret;
}

/**
 * @brief 设置吸盘最小真空度
 * @return MODBUS_OK 成功，其他错误码
 */
int suction_cup_set_min_vacuum(uint16_t vacuum)
{
    return modbus_write_single_register(MODBUS_ADDR_SUCTION_CUP, 0x0003, vacuum);
}

/**
 * @brief 设置吸盘最大真空度
 * @return MODBUS_OK 成功，其他错误码
 */
int suction_cup_set_max_vacuum(uint16_t vacuum)
{
    return modbus_write_single_register(MODBUS_ADDR_SUCTION_CUP, 0x0004, vacuum);
}

/**
 * @brief 吸盘吸取 (简单吸取模式)
 * @return MODBUS_OK 成功，其他错误码
 */
int suction_cup_suck(void)
{
    return modbus_write_single_register(MODBUS_ADDR_SUCTION_CUP, 0x0002, SUCTION_CMD_SUCK);
}

/**
 * @brief 吸盘释放
 * @return MODBUS_OK 成功，其他错误码
 */
int suction_cup_release(void)
{
    return modbus_write_single_register(MODBUS_ADDR_SUCTION_CUP, 0x0002, SUCTION_CMD_RELEASE);
}

/**
 * @brief 按真空度范围吸取 (使用功能码0x10连续写入3个寄存器)
 * @param min_vacuum 最小真空度 (1-50)
 * @param max_vacuum 最大真空度 (1-50)
 * @return MODBUS_OK 成功，其他错误码
 */
int suction_cup_vacuum_suck(uint16_t min_vacuum, uint16_t max_vacuum)
{
    uint16_t values[3];
    values[0] = SUCTION_CMD_VACUUM_SUCK;
    values[1] = min_vacuum;
    values[2] = max_vacuum;
    return modbus_write_multiple_registers(MODBUS_ADDR_SUCTION_CUP, 0x0002, 3, values);
}

/**
 * @brief 读取吸盘吸取状态
 * @param state 输出状态: 0=释放/初始化, 1=吸住, 2=未吸住, 3=掉落
 * @return MODBUS_OK 成功，其他错误码
 */
int suction_cup_read_state(SuctionCupState_t *state)
{
    uint16_t reg = 0;
    int ret = modbus_read_holding_registers(MODBUS_ADDR_SUCTION_CUP, 0x0041, 1, &reg);
    if (ret == MODBUS_OK) {
        *state = (SuctionCupState_t)reg;
    }
    return ret;
}

/**
 * @brief 读取当前真空度反馈
 * @param vacuum 输出真空度值
 * @return MODBUS_OK 成功，其他错误码
 */
int suction_cup_read_vacuum(uint16_t *vacuum)
{
    return modbus_read_holding_registers(MODBUS_ADDR_SUCTION_CUP, 0x0042, 1, vacuum);
}

/**
 * @brief 设置I/O模式开关
 * @param enable 0=关闭I/O模式, 1=打开I/O模式
 * @return MODBUS_OK 成功，其他错误码
 */
int suction_cup_set_io_mode(uint16_t enable)
{
    return modbus_write_single_register(MODBUS_ADDR_SUCTION_CUP, 0x0090, enable);
}

/**
 * @brief 设置吸取判断时间
 * @param seconds 判断时间, 单位秒, 默认2s, 应>=1s
 * @return MODBUS_OK 成功，其他错误码
 */
int suction_cup_set_judge_time(uint16_t seconds)
{
    return modbus_write_single_register(MODBUS_ADDR_SUCTION_CUP, 0x0091, seconds);
}

/**
 * @brief 配置点位参数 (吸/放动作和真空度范围)
 * @param point 点位编号 (0~3对应点位1~4)
 * @param cmd 控制指令: 1=吸取, 2=释放
 * @param min_vacuum 最小真空度 (1-50)
 * @param max_vacuum 最大真空度 (1-50)
 * @return MODBUS_OK 成功，其他错误码
 */
int suction_cup_set_point_config(SuctionPoint_t point, SuctionCmd_t cmd,
                                  uint16_t min_vacuum, uint16_t max_vacuum)
{
    uint16_t values[3];
    uint16_t base_addr;

    base_addr = SUCTION_POINT_BASE_ADDR + (uint16_t)point * SUCTION_POINT_REGS_PER;
    values[0] = (uint16_t)cmd;
    values[1] = min_vacuum;
    values[2] = max_vacuum;

    return modbus_write_multiple_registers(MODBUS_ADDR_SUCTION_CUP, base_addr, 3, values);
}

/**
 * @brief 保存参数到Flash (断电保持)
 * @return MODBUS_OK 成功，其他错误码
 */
int suction_cup_save_params(void)
{
    return modbus_write_single_register(MODBUS_ADDR_SUCTION_CUP, 0x0084, 0x0001);
}

/**
 * @brief 读取设备信息 (型号/硬件版本/软件版本)
 * @param model 输出吸盘型号
 * @param hw_ver 输出硬件版本
 * @param sw_ver 输出软件版本
 * @return MODBUS_OK 成功，其他错误码
 */
int suction_cup_read_device_info(uint16_t *model, uint16_t *hw_ver, uint16_t *sw_ver)
{
    uint16_t reg_buf[3] = {0};
    int ret = modbus_read_holding_registers(MODBUS_ADDR_SUCTION_CUP, 0x00C0, 3, reg_buf);
    if (ret == MODBUS_OK) {
        *model = reg_buf[0];
        *hw_ver = reg_buf[1];
        *sw_ver = reg_buf[2];
    }
    return ret;
}

/* ===================================================================
 *           动作到位检查 & 阻塞等待接口
 * =================================================================== */

#define SUC_POLL_INTERVAL_MS    50   // 轮询间隔

/**
 * @brief 检查吸盘当前状态（单次查询）
 */
bool suction_cup_check_state(SuctionCupState_t *state)
{
    if (state == NULL) return false;
    return (suction_cup_read_state(state) == MODBUS_OK);
}

/**
 * @brief 检查吸盘是否已吸住物体
 */
bool suction_cup_is_holding(void)
{
    SuctionCupState_t state;
    if (suction_cup_read_state(&state) != MODBUS_OK) return false;
    return (state == SUCTION_CUP_HOLDING);
}

/**
 * @brief 检查吸盘是否已释放
 */
bool suction_cup_is_released(void)
{
    SuctionCupState_t state;
    if (suction_cup_read_state(&state) != MODBUS_OK) return false;
    return (state == SUCTION_CUP_RELEASE);
}

/**
 * @brief 阻塞等待吸盘达到期望状态
 * @return 0=达到, -1=超时, -2=通讯失败
 */
int suction_cup_wait_state(SuctionCupState_t desired_state, uint32_t timeout_ms)
{
    SuctionCupState_t state;
    uint32_t elapsed = 0;

    while (elapsed < timeout_ms) {
        if (modbus_reg_is_stop_requested()) {
            return ACTION_WAIT_CANCELLED;
        }
        osDelay(SUC_POLL_INTERVAL_MS);             // 先等硬件反应，再读状态
        elapsed += SUC_POLL_INTERVAL_MS;

        if (suction_cup_read_state(&state) != MODBUS_OK) {
            return -2;  // 通讯失败
        }
        if (state == desired_state) {
            uint32_t stable_ms = 0;
            while (stable_ms < 1000) {
                if (modbus_reg_is_stop_requested()) {
                    return ACTION_WAIT_CANCELLED;
                }
                osDelay(SUC_POLL_INTERVAL_MS);
                stable_ms += SUC_POLL_INTERVAL_MS;
            }
            return 0;   // 达到期望状态
        }
        // 检测异常终态：掉落 (3)、未吸住 (2) — 不再等待
        if (desired_state == SUCTION_CUP_HOLDING &&
            (state == SUCTION_CUP_DROPPED || state == SUCTION_CUP_NOT_HOLD)) {
            printf("[SUC] Wait hold failed: state=%d\r\n", state);
            return -2;
        }
    }

    printf("[SUC] Wait state=%d timeout (%lums)\r\n", desired_state, timeout_ms);
    return -1;
}

/**
 * @brief 阻塞等待吸盘吸住物体
 */
int suction_cup_wait_hold(uint32_t timeout_ms)
{
    return suction_cup_wait_state(SUCTION_CUP_HOLDING, timeout_ms);
}

/**
 * @brief 阻塞等待吸盘完全释放
 */
int suction_cup_wait_release(uint32_t timeout_ms)
{
    return suction_cup_wait_state(SUCTION_CUP_RELEASE, timeout_ms);
}
