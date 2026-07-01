#include "dev_cylinder.h"
#include "modbus_rtu.h"
#include "app_control.h"
#include "modbus_slave_reg.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdio.h>

/* ------------------------------------------------------------------ *
 *  内部工具：电缸ID → Modbus 从机地址
 * ------------------------------------------------------------------ */
static uint8_t cylinder_id_to_addr(CylinderId_t id)
{
    switch (id) {
        case CYLINDER_ID_SUCK:
            return MODBUS_ADDR_SUCK_CYLINDER;   /* 0x02 */
        case CYLINDER_ID_SEAL:
            return MODBUS_ADDR_PAVE_CYLINDER;   /* 0x03 */
        default:
            return 0;
    }
}

/* ------------------------------------------------------------------ *
 *  双电缸初始化（依次配置并回零两个电缸）
 * ------------------------------------------------------------------ */
/**
 * @brief 双电缸上电初始化：依次归零并阻塞等待完成
 * @return 0=成功, -1=任一电缸失败
 *
 * 与 dev_fujun_motor_init 同模式：阻塞式初始化，成功返回0，
 * 调用方无需再单独处理回零等待。
 */
int dev_cylinder_init(void)
{
    /* ---- 吸膜电缸归零 ---- */
    if (cylinder_home(CYLINDER_ID_SUCK) != MODBUS_OK) {
        printf("[CYL] Suction cylinder home cmd FAIL\r\n");
        return -1;
    }
    if (cylinder_wait_home(CYLINDER_ID_SUCK, 30000) != 0) {
        printf("[CYL] Suction cylinder home timeout/FAIL\r\n");
        return -1;
    }
    printf("[CYL] Suction cylinder home OK\r\n");

    /* ---- 封膜电缸归零 ---- */
    if (cylinder_home(CYLINDER_ID_SEAL) != MODBUS_OK) {
        printf("[CYL] Seal cylinder home cmd FAIL\r\n");
        return -1;
    }
    if (cylinder_wait_home(CYLINDER_ID_SEAL, 30000) != 0) {
        printf("[CYL] Seal cylinder home timeout/FAIL\r\n");
        return -1;
    }
    printf("[CYL] Seal cylinder home OK\r\n");

    return 0;
}

/* ------------------------------------------------------------------ *
 *  控制类 API（写寄存器）
 * ------------------------------------------------------------------ */

/**
 * @brief 电缸回零
 * @param id 电缸 ID
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_home(CylinderId_t id)
{
    return modbus_write_single_register(cylinder_id_to_addr(id), 0x0100, 0x0001);
}

/**
 * @brief 设置电缸推压力值
 * @param id 电缸 ID
 * @param force_percent 力值百分比 (1-100)
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_set_force(CylinderId_t id, uint16_t force_percent)
{
    return modbus_write_single_register(cylinder_id_to_addr(id), 0x0101, force_percent);
}

/**
 * @brief 设置推压段总长度
 * @param id 电缸 ID
 * @param length_001mm 推压段长度, 单位0.01mm
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_set_push_length(CylinderId_t id, uint16_t length_001mm)
{
    return modbus_write_single_register(cylinder_id_to_addr(id), 0x0102, length_001mm);
}

/**
 * @brief 运动到指定位置
 * @param id 电缸 ID
 * @param position_001mm 目标位置, 单位0.01mm
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_move_to(CylinderId_t id, uint16_t position_001mm)
{
    return modbus_write_single_register(cylinder_id_to_addr(id), 0x0103, position_001mm);
}

/**
 * @brief 设置运动段最大速度
 * @param id 电缸 ID
 * @param speed_percent 速度百分比 (1-100)
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_set_max_speed(CylinderId_t id, uint16_t speed_percent)
{
    return modbus_write_single_register(cylinder_id_to_addr(id), 0x0104, speed_percent);
}

/**
 * @brief 设置加/减速度
 * @param id 电缸 ID
 * @param accel_percent 加/减速度百分比 (1-100)
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_set_accel(CylinderId_t id, uint16_t accel_percent)
{
    return modbus_write_single_register(cylinder_id_to_addr(id), 0x0105, accel_percent);
}

/**
 * @brief 相对当前位置移动
 * @param id 电缸 ID
 * @param offset_001mm 相对偏移量, 单位0.01mm, 范围-32767~32767
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_move_relative(CylinderId_t id, int16_t offset_001mm)
{
    return modbus_write_single_register(cylinder_id_to_addr(id), 0x0106, (uint16_t)offset_001mm);
}

/**
 * @brief 电缸点动 (JOG模式)
 * @param id 电缸 ID
 * @param dir 方向: -1=反向, 0=停止, 1=正向
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_jog(CylinderId_t id, CylinderJogDir_t dir)
{
    return modbus_write_single_register(cylinder_id_to_addr(id), 0x0107, (uint16_t)dir);
}

/* ------------------------------------------------------------------ *
 *  状态读取类 API（读寄存器）
 * ------------------------------------------------------------------ */

/**
 * @brief 读取回零状态
 * @param id 电缸 ID
 * @param state 输出回零状态: 0=未回零, 1=已回零成功, 2=回零中
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_read_home_state(CylinderId_t id, CylinderHomeState_t *state)
{
    uint16_t reg = 0;
    int ret = modbus_read_holding_registers(cylinder_id_to_addr(id), 0x0200, 1, &reg);
    if (ret == MODBUS_OK) {
        *state = (CylinderHomeState_t)reg;
    }
    return ret;
}

/**
 * @brief 读取电缸运动状态
 * @param id 电缸 ID
 * @param state 输出运动状态: 0=运动中, 1=到达位置, 2=堵转
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_read_motion_state(CylinderId_t id, CylinderMotionState_t *state)
{
    uint16_t reg = 0;
    int ret = modbus_read_holding_registers(cylinder_id_to_addr(id), 0x0201, 1, &reg);
    if (ret == MODBUS_OK) {
        *state = (CylinderMotionState_t)reg;
    }
    return ret;
}

/**
 * @brief 读取当前实时位置
 * @param id 电缸 ID
 * @param position_001mm 输出当前位置, 单位0.01mm
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_read_position(CylinderId_t id, uint16_t *position_001mm)
{
    return modbus_read_holding_registers(cylinder_id_to_addr(id), 0x0202, 1, position_001mm);
}

/**
 * @brief 读取当前电流
 * @param id 电缸 ID
 * @param current 输出电流值
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_read_current(CylinderId_t id, uint16_t *current)
{
    return modbus_read_holding_registers(cylinder_id_to_addr(id), 0x0204, 1, current);
}

/* ------------------------------------------------------------------ *
 *  参数管理类 API
 * ------------------------------------------------------------------ */

/**
 * @brief 保存参数到Flash
 * @param id 电缸 ID
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_save_params(CylinderId_t id)
{
    return modbus_write_single_register(cylinder_id_to_addr(id), 0x0300, 0x0001);
}

/**
 * @brief 设置电缸回零方向
 * @param id 电缸 ID
 * @param dir 回零方向: 0=前进归零, 1=后退归零
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_set_home_dir(CylinderId_t id, CylinderHomeDir_t dir)
{
    return modbus_write_single_register(cylinder_id_to_addr(id), 0x0301, (uint16_t)dir);
}

/**
 * @brief 设置推压段速度
 * @param id 电缸 ID
 * @param speed_percent 推压速度百分比 (1-100), 默认20
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_set_push_speed(CylinderId_t id, uint16_t speed_percent)
{
    return modbus_write_single_register(cylinder_id_to_addr(id), 0x0309, speed_percent);
}

/**
 * @brief 设置推压方向
 * @param id 电缸 ID
 * @param dir 推压方向: 0=正向, 1=反向
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_set_push_dir(CylinderId_t id, CylinderPushDir_t dir)
{
    return modbus_write_single_register(cylinder_id_to_addr(id), 0x030A, (uint16_t)dir);
}

/**
 * @brief IO参数测试: 直接控制4组预设IO参数
 * @param id 电缸 ID
 * @param group IO组号 (1~4)
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_io_test(CylinderId_t id, uint16_t group)
{
    return modbus_write_single_register(cylinder_id_to_addr(id), 0x0400, group);
}

/**
 * @brief 设置I/O模式开关
 * @param id 电缸 ID
 * @param enable 0=关闭I/O模式, 1=开启I/O模式
 * @return MODBUS_OK 成功，其他错误码
 */
int cylinder_set_io_mode(CylinderId_t id, uint16_t enable)
{
    return modbus_write_single_register(cylinder_id_to_addr(id), 0x0402, enable);
}

/* ===================================================================
 *           动作到位检查 & 阻塞等待接口
 * =================================================================== */

#define CYL_POLL_INTERVAL_MS   100   // 轮询间隔
#define CYL_CMD_SETTLE_MS      150   // 给电缸控制器刷新运动状态的时间，避免读到上一轮到位状态
#define CYL_POS_TOLERANCE      20    // 位置到位容差，单位0.01mm，20=0.20mm
#define CYL_ARRIVED_CONFIRM    2     // 状态到位+位置到位连续确认次数

static uint16_t cylinder_abs_diff_u16(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

/* ---- 单次查询 ---- */

/**
 * @brief 检查电缸是否已到达目标位置
 */
bool cylinder_is_arrived(CylinderId_t id)
{
    CylinderMotionState_t state;
    if (cylinder_read_motion_state(id, &state) != MODBUS_OK) return false;
    return (state == CYLINDER_MOTION_ARRIVED);
}

/**
 * @brief 检查电缸是否仍在运动中
 */
bool cylinder_is_moving(CylinderId_t id)
{
    CylinderMotionState_t state;
    if (cylinder_read_motion_state(id, &state) != MODBUS_OK) return false;
    return (state == CYLINDER_MOTION_RUNNING);
}

/**
 * @brief 检查电缸是否堵转
 */
bool cylinder_is_stalled(CylinderId_t id)
{
    CylinderMotionState_t state;
    if (cylinder_read_motion_state(id, &state) != MODBUS_OK) return false;
    return (state == CYLINDER_MOTION_STALLED);
}

/**
 * @brief 检查电缸是否已回零成功
 */
bool cylinder_is_homed(CylinderId_t id)
{
    CylinderHomeState_t state;
    if (cylinder_read_home_state(id, &state) != MODBUS_OK) return false;
    return (state == CYLINDER_HOME_SUCCESS);
}

/* ---- 阻塞等待 ---- */

/**
 * @brief 阻塞等待电缸运动到目标位置
 * @return 0=已到位, -1=超时, -2=堵转, -3=通讯失败
 */
int cylinder_wait_arrived(CylinderId_t id, uint32_t timeout_ms)
{
    CylinderMotionState_t state;
    uint32_t elapsed = 0;

    while (elapsed < timeout_ms) {
        if (modbus_reg_is_stop_requested()) {
            return ACTION_WAIT_CANCELLED;
        }
        int ret = cylinder_read_motion_state(id, &state);
        if (ret != MODBUS_OK) {
            return -3;  // 通讯失败
        }
        if (state == CYLINDER_MOTION_ARRIVED) {
            return 0;   // 已到位
        }
        if (state == CYLINDER_MOTION_STALLED) {
            printf("[CYL] id=%d stalled!\r\n", (int)id);
            return -2;  // 堵转
        }
        osDelay(CYL_POLL_INTERVAL_MS);
        elapsed += CYL_POLL_INTERVAL_MS;
    }

    printf("[CYL] id=%d wait arrived timeout (%lums)\r\n", (int)id, timeout_ms);
    return -1;
}

/**
 * @brief 等待电缸状态到位且当前位置到达目标位置
 * @return 0=目标位置到位, -1=超时, -2=堵转, -3=通讯失败, ACTION_WAIT_CANCELLED=被停止命令取消
 */
int cylinder_wait_position(CylinderId_t id, uint16_t target_001mm, uint32_t timeout_ms)
{
    CylinderMotionState_t state = CYLINDER_MOTION_RUNNING;
    uint16_t position = 0xFFFF;
    uint16_t diff = 0xFFFF;
    uint32_t elapsed = 0;
    uint8_t confirm = 0;

    while (elapsed < CYL_CMD_SETTLE_MS && elapsed < timeout_ms) {
        if (modbus_reg_is_stop_requested()) {
            return ACTION_WAIT_CANCELLED;
        }
        uint32_t settle_left = CYL_CMD_SETTLE_MS - elapsed;
        uint32_t timeout_left = timeout_ms - elapsed;
        uint32_t slice = (settle_left > 50U) ? 50U : settle_left;
        if (slice > timeout_left) {
            slice = timeout_left;
        }
        osDelay(slice);
        elapsed += slice;
    }

    while (elapsed < timeout_ms) {
        if (modbus_reg_is_stop_requested()) {
            return ACTION_WAIT_CANCELLED;
        }

        int ret = cylinder_read_motion_state(id, &state);
        if (ret != MODBUS_OK) {
            printf("[CYL] id=%d read motion state FAIL ret=%d\r\n", (int)id, ret);
            return -3;
        }

        ret = cylinder_read_position(id, &position);
        if (ret != MODBUS_OK) {
            printf("[CYL] id=%d read position FAIL ret=%d\r\n", (int)id, ret);
            return -3;
        }

        diff = cylinder_abs_diff_u16(position, target_001mm);

        if (state == CYLINDER_MOTION_STALLED) {
            printf("[CYL] id=%d stalled! target=%u pos=%u diff=%u\r\n",
                   (int)id, target_001mm, position, diff);
            return -2;
        }

        if (state == CYLINDER_MOTION_ARRIVED && diff <= CYL_POS_TOLERANCE) {
            confirm++;
            if (confirm >= CYL_ARRIVED_CONFIRM) {
                printf("[CYL] id=%d arrived target=%u pos=%u diff=%u\r\n",
                       (int)id, target_001mm, position, diff);
                return 0;
            }
        } else {
            confirm = 0;
        }

        osDelay(CYL_POLL_INTERVAL_MS);
        elapsed += CYL_POLL_INTERVAL_MS;
    }

    printf("[CYL] id=%d wait target timeout target=%u pos=%u diff=%u state=%d (%lums)\r\n",
           (int)id, target_001mm, position, diff, (int)state, timeout_ms);
    return -1;
}

/**
 * @brief 下发目标位置并等待实际到位
 */
int cylinder_move_to_wait(CylinderId_t id, uint16_t position_001mm, uint32_t timeout_ms)
{
    int ret = cylinder_move_to(id, position_001mm);
    if (ret != MODBUS_OK) {
        printf("[CYL] id=%d move cmd FAIL target=%u ret=%d\r\n",
               (int)id, position_001mm, ret);
        return -3;
    }

    return cylinder_wait_position(id, position_001mm, timeout_ms);
}

/**
 * @brief 阻塞等待电缸回零完成
 * @return 0=回零成功, -1=超时, -2=通讯失败
 */
int cylinder_wait_home(CylinderId_t id, uint32_t timeout_ms)
{
    CylinderHomeState_t state;
    uint32_t elapsed = 0;

    while (elapsed < timeout_ms) {
        if (modbus_reg_is_stop_requested()) {
            return ACTION_WAIT_CANCELLED;
        }
        int ret = cylinder_read_home_state(id, &state);
        if (ret != MODBUS_OK) {
            return -2;  // 通讯失败
        }
        if (state == CYLINDER_HOME_SUCCESS) {
            return 0;   // 回零成功
        }
        osDelay(CYL_POLL_INTERVAL_MS);
        elapsed += CYL_POLL_INTERVAL_MS;
    }

    printf("[CYL] id=%d wait home timeout (%lums)\r\n", (int)id, timeout_ms);
    return -1;
}
