#include "dev_fujun_motor.h"
#include "modbus_rtu.h"
#include "app_control.h"
#include "modbus_slave_reg.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <stdlib.h>

/* ------------------------------------------------------------------ *
 *  富俊步进电机驱动 (基于 485/Modbus RTU, 手册 SV126.1)
 *
 *  寄存器地址速查:
 *    0x0004  实时位置     INT32  RO  pulses
 *    0x009B  限位端口设置 UINT16 RW  -
 *    0x00A3  读取报警     UINT16 RO  -
 *    0x00A4  清除报警     UINT16 WO  -
 *    0x00CE  运行脉冲数   INT32  WO  pulses
 *    0x00D0  绝对位置运行 INT32  WO  pulses
 *    0x00D2  设置当前位置 INT32  WO  pulses
 *    0x00D4  脱机/使能    UINT16 WO  -
 *    0x00D7  旋转方向     UINT16 WO  0=正向, 1=反向
 *    0x00D8  运行速度     INT32  RW  0.01rpm
 *    0x00DC  断电保存     UINT16 WO  -
 * ------------------------------------------------------------------ */

/* ---------- 内部辅助 ---------- */

static int write_int32(uint16_t reg, int32_t value)
{
    uint16_t v[2];
    v[0] = (uint16_t)(value & 0xFFFF);           // 低16位在前
    v[1] = (uint16_t)((uint32_t)value >> 16);     // 高16位在后
    return modbus_write_multiple_registers(MODBUS_ADDR_FUJUN_MOTOR, reg, 2, v);
}

static int read_int32(uint16_t reg, int32_t *value)
{
    uint16_t buf[2] = {0};
    int ret = modbus_read_holding_registers(MODBUS_ADDR_FUJUN_MOTOR, reg, 2, buf);
    if (ret == MODBUS_OK) {
        *value = (int32_t)((uint32_t)buf[0] | ((uint32_t)buf[1] << 16));  // buf[0]=低字, buf[1]=高字
    }
    return ret;
}

static int write_reg(uint16_t reg, uint16_t value)
{
    return modbus_write_single_register(MODBUS_ADDR_FUJUN_MOTOR, reg, value);
}

static int read_reg(uint16_t reg, uint16_t *value)
{
    return modbus_read_holding_registers(MODBUS_ADDR_FUJUN_MOTOR, reg, 1, value);
}

/* ---------- 初始化 ---------- */

int dev_fujun_motor_init(void)
{
    // 1. 电机使能
    if (modbus_write_single_register(MODBUS_ADDR_FUJUN_MOTOR, 0x00D4, 0x0000) != MODBUS_OK) {
        return -1;
    }

    // 2. 设置正负限位端口 (0x009B)
    //     pos: 正限位, NPN, X0
    //     neg: 负限位, NPN, X1
    if (fujun_motor_set_limit_ports(1, 1, 1, 1, 1, 0) != MODBUS_OK) {
        return -1;
    }

    // 2.5. 设置旋转方向为正向
    if (fujun_motor_set_direction(1) != MODBUS_OK) {
        return -1;
    }

    // 3. 相对运行 10000 脉冲，消除报警
    if (write_int32(0x00CE, 10000) != MODBUS_OK) {
        return -1;
    }
    osDelay(2000);

    // 3.1 清除报警
    if (write_reg(0x00A4, 0x0001) != MODBUS_OK) {
        return -1;
    }
    osDelay(50);

    // 3.2. 相对运行 -110000 脉冲,超过最大限程 (撞限位归零)
    if (write_int32(0x00CE, -110000) != MODBUS_OK) {
        return -1;
    }

    // 3.3 等待撞限位完成
    osDelay(5000);

    // 3.4 清除撞限位产生的报警 (否则历史报警会残留，干扰后续判断)
    if (write_reg(0x00A4, 0x0001) != MODBUS_OK) {
        return -1;
    }
    osDelay(50);

    // 4. 设置当前位置为零点
    if (write_int32(0x00D2, 0) != MODBUS_OK) {
        return -1;
    }

    // 5. 断电保存
    if (write_reg(0x00DC, 0x0001) != MODBUS_OK) {
        return -1;
    }

    return 0;
}

/* ---------- 设置速度 ---------- */

int fujun_motor_set_speed(int32_t speed_001rpm)
{
    return write_int32(0x00D8, speed_001rpm);
}

/* ---------- 硬件限位端口设置 ---------- */

int fujun_motor_set_limit_ports(uint8_t pos_enable, uint8_t pos_signal, uint8_t pos_port,
                                 uint8_t neg_enable, uint8_t neg_signal, uint8_t neg_port)
{
    uint16_t reg_val = 0;

    // 正限位: BIT[7:5]=enable, BIT[4]=signal, BIT[3:0]=port
    if (pos_enable) reg_val |= (1 << 5);
    if (pos_signal)  reg_val |= (1 << 4);
    reg_val |= (pos_port & 0x0F);

    // 负限位: BIT[15:13]=enable, BIT[12]=signal, BIT[11:8]=port
    if (neg_enable) reg_val |= (1 << 13);
    if (neg_signal)  reg_val |= (1 << 12);
    reg_val |= ((neg_port & 0x0F) << 8);

    return write_reg(0x009B, reg_val);
}

/* ---------- 设置当前绝对位置 / 零点 ---------- */

int fujun_motor_set_current_position(int32_t position)
{
    return write_int32(0x00D2, position);
}

/* ---------- 运行脉冲数 ---------- */

int fujun_motor_run_pulses(int32_t pulses)
{
    return write_int32(0x00CE, pulses);
}

/* ---------- 运行到绝对位置 ---------- */

int fujun_motor_move_to_absolute(int32_t position)
{
    return write_int32(0x00D0, position);
}

/* ---------- 断电保存 ---------- */

int fujun_motor_save_params(void)
{
    return write_reg(0x00DC, 0x0001);
}

/* ---------- 读取报警 ---------- */

int fujun_motor_read_alarm(uint16_t *alarm)
{
    if (alarm == NULL) return MODBUS_ERR_PARM;
    return read_reg(0x00A3, alarm);
}

/* ---------- 清除报警 ---------- */

int fujun_motor_clear_alarm(void)
{
    return write_reg(0x00A4, 0x0001);
}

/**
 * @brief 设置电机旋转方向
 * @param dir 0=正向(默认), 1=反向
 */
int fujun_motor_set_direction(uint16_t dir)
{
    return write_reg(0x006B, dir);
}

/* ---------- 查询实时位置 ---------- */

int fujun_motor_read_position(int32_t *position)
{
    if (position == NULL) return MODBUS_ERR_PARM;
    return read_int32(0x0004, position);
}

/* ===================================================================
 *           动作到位检查 & 阻塞等待接口
 * =================================================================== */

#define FUJUN_POLL_INTERVAL_MS     200   // 轮询间隔
#define FUJUN_POSITION_TOLERANCE   40    // 到位判定误差 (pulses)
#define FUJUN_WAIT_TIMEOUT_MS    10000  // 等待到位最大时间 (ms)

/* ---- 状态寄存器读取 ---- */

/**
 * @brief 读取电机状态寄存器 (0x00A0)
 */
int fujun_motor_read_status(uint16_t *status)
{
    if (status == NULL) return MODBUS_ERR_PARM;
    return read_reg(0x00A0, status);
}

/* ---- 单次状态查询 ---- */

/**
 * @brief 检查电机是否正在运行
 */
bool fujun_motor_is_running(void)
{
    uint16_t status;
    if (fujun_motor_read_status(&status) != MODBUS_OK) return false;
    return (status & FUJUN_STATUS_RUNNING) != 0;
}

/**
 * @brief 检查电机是否已停止
 */
bool fujun_motor_is_stopped(void)
{
    uint16_t status;
    if (fujun_motor_read_status(&status) != MODBUS_OK) return false;
    return (status & FUJUN_STATUS_RUNNING) == 0;
}

/**
 * @brief 检查电机是否有报警
 */
bool fujun_motor_has_alarm(void)
{
    uint16_t alarm = 0;
    if (fujun_motor_read_alarm(&alarm) != MODBUS_OK) return false;
    return (alarm & 0x000F) != 0;   // 只检查当前报警 (bits[3:0])
}

/**
 * @brief 检查电机是否已到达目标位置
 */
bool fujun_motor_is_arrived(void)
{
    uint16_t status;
    if (fujun_motor_read_status(&status) != MODBUS_OK) return false;
    return ((status & FUJUN_STATUS_RUNNING) == 0) && ((status & FUJUN_STATUS_READY) != 0);
}

/* ---- 阻塞等待 ---- */

/**
 * @brief 阻塞等待电机停止运行
 * @return 0=已停止, -1=超时, -2=报警, -3=通讯失败
 */
int fujun_motor_wait_stop(uint32_t timeout_ms)
{
    uint32_t elapsed = 0;

    while (elapsed < timeout_ms) {
        if (modbus_reg_is_stop_requested()) {
            return ACTION_WAIT_CANCELLED;
        }
        uint16_t status;
        if (fujun_motor_read_status(&status) != MODBUS_OK) {
            return -3;  // 通讯失败
        }

        // 检查当前报警 (bits[3:0]), 忽略历史报警
        if (status & FUJUN_STATUS_ALARM) {
            uint16_t alarm = 0;
            fujun_motor_read_alarm(&alarm);
            printf("[FUJUN] Alarm detected: 0x%04X (current=%d)\r\n",
                   alarm, (int)(alarm & 0x000F));
            return -2;
        }

        // 检查是否已停止
        if ((status & FUJUN_STATUS_RUNNING) == 0) {
            return 0;
        }

        osDelay(FUJUN_POLL_INTERVAL_MS);
        elapsed += FUJUN_POLL_INTERVAL_MS;
    }

    printf("[FUJUN] Wait stop timeout (%lums)\r\n", timeout_ms);
    return -1;
}

/**
 * @brief 移动电机到目标位置并等待到达
 * @return 0=已到达, -1=超时, -2=报警, -3=通讯失败
 *
 * 到位误差容限 FUJUN_POSITION_TOLERANCE，最大等待时长 FUJUN_WAIT_TIMEOUT_MS
 */
int fujun_motor_wait_position(int32_t target)
{
    int32_t cur_pos;
    const int MAX_ATTEMPTS = 4;  // 1 initial + 3 retries

    /*
     * 到位判断策略：
     *   1. 读取当前位置，计算剩余误差 delta = target - current
     *   2. 若误差在 FUJUN_POSITION_TOLERANCE 内，直接判定到位
     *   3. 若误差超限，使用 0x00CE 相对运行剩余脉冲数进行补偿
     *   4. 等待结束后再次读取位置判断，不到位则继续按剩余误差相对补偿
     */
    for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
        if (modbus_reg_is_stop_requested()) {
            return ACTION_WAIT_CANCELLED;
        }

        /* 读取当前位置，并按剩余误差决定相对补偿量 */
        if (fujun_motor_read_position(&cur_pos) != MODBUS_OK) {
            osDelay(200);
            continue;
        }

        int32_t delta = target - cur_pos;
        int32_t diff = abs(delta);
        if (diff <= FUJUN_POSITION_TOLERANCE) {
            return 0;
        }

        /* 使用相对运行补偿剩余误差，避免绝对位置小偏差时驱动器不动作 */
        if (fujun_motor_run_pulses(delta) != MODBUS_OK) {
            osDelay(200);
            continue;
        }

        printf("[FUJUN] Correction move: target=%ld current=%ld delta=%ld (attempt %d/%d)\r\n",
               (long)target, (long)cur_pos, (long)delta, attempt + 1, MAX_ATTEMPTS);

        /* 按距离换算等待时间：20000 pulses/s + 2s 余量 */
        uint32_t distance = (uint32_t)diff;
        uint32_t wait_ms = distance / 20 + 2000;
        if (wait_ms > FUJUN_WAIT_TIMEOUT_MS) {
            wait_ms = FUJUN_WAIT_TIMEOUT_MS;
        }

        uint32_t waited_ms = 0;
        while (waited_ms < wait_ms) {
            uint32_t slice_ms = (wait_ms - waited_ms > FUJUN_POLL_INTERVAL_MS)
                              ? FUJUN_POLL_INTERVAL_MS : (wait_ms - waited_ms);
            if (modbus_reg_is_stop_requested()) {
                return ACTION_WAIT_CANCELLED;
            }
            osDelay(slice_ms);
            waited_ms += slice_ms;
        }

        /* 检查当前报警 (bits[3:0]), 忽略历史报警 (bits[15:4]) */
        uint16_t alarm = 0;
        if (fujun_motor_read_alarm(&alarm) == MODBUS_OK && (alarm & 0x000F) != 0) {
            printf("[FUJUN] Alarm detected: 0x%04X (current=%d), clearing...\r\n",
                   alarm, (int)(alarm & 0x000F));
            fujun_motor_clear_alarm();
            return -2;
        }

        /* 读取最终位置并判断 */
        if (fujun_motor_read_position(&cur_pos) == MODBUS_OK) {
            int32_t final_diff = abs(cur_pos - target);
            if (final_diff <= FUJUN_POSITION_TOLERANCE) {
                return 0;
            }
            printf("[FUJUN] Position mismatch: target=%ld actual=%ld diff=%ld (attempt %d/%d)\r\n",
                   (long)target, (long)cur_pos, (long)final_diff, attempt + 1, MAX_ATTEMPTS);
        }
    }

    printf("[FUJUN] Failed to reach target=%ld after %d attempts\r\n",
           (long)target, MAX_ATTEMPTS);
    return -1;
}
