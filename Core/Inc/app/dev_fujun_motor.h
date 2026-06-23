#ifndef __DEV_FUJUN_MOTOR_H
#define __DEV_FUJUN_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

int dev_fujun_motor_init(void);

// 设置速度 (0x00D8~0x00D9, 单位: 0.01 rpm)
int fujun_motor_set_speed(int32_t speed_001rpm);

// 硬件限位端口设置 (0x009B)
//   pos/neg_enable: 0=取消, 1=设置
//   pos/neg_signal: 0=PNP(低有效), 1=NPN(高有效)
//   pos/neg_port:   输入端口号 0~15 (X0~X15)
int fujun_motor_set_limit_ports(uint8_t pos_enable, uint8_t pos_signal, uint8_t pos_port,
                                 uint8_t neg_enable, uint8_t neg_signal, uint8_t neg_port);

// 设置当前电机绝对位置/零点 (0x00D2~0x00D3, 单位: pulses)
int fujun_motor_set_current_position(int32_t position);

// 运行指定脉冲数 (0x00CE~0x00CF, 相对当前位置, 仅停止时可执行)
int fujun_motor_run_pulses(int32_t pulses);

// 运行到绝对位置 (0x00D0~0x00D1, 单位: pulses, 仅停止时可执行)
int fujun_motor_move_to_absolute(int32_t position);

// 断电保存 (0x00DC)
int fujun_motor_save_params(void);

// 读取报警状态 (0x00A3)
int fujun_motor_read_alarm(uint16_t *alarm);

// 清除报警 (0x00A4)
int fujun_motor_clear_alarm(void);

// 设置电机旋转方向 (0x00D7, 0=正向, 1=反向)
int fujun_motor_set_direction(uint16_t dir);

// 查询电机实时位置 (0x0004~0x0005, 单位: pulses)
int fujun_motor_read_position(int32_t *position);

/* ------------------------------------------------------------------ *
 *  动作到位检查 & 等待接口
 * ------------------------------------------------------------------ */

// 电机状态寄存器 (0x00A0) 位定义
#define FUJUN_STATUS_RUNNING        (1 << 0)   // 电机运行中
#define FUJUN_STATUS_READY          (1 << 1)   // 电机就绪
#define FUJUN_STATUS_ALARM          (1 << 2)   // 报警标志
#define FUJUN_STATUS_ARRIVED        (1 << 3)   // 到达目标位置
#define FUJUN_STATUS_POS_LIMIT      (1 << 4)   // 正限位触发
#define FUJUN_STATUS_NEG_LIMIT      (1 << 5)   // 负限位触发

/**
 * @brief 读取电机状态寄存器 (0x00A0)
 * @param status 输出状态字（按 FUJUN_STATUS_* 位定义）
 * @return MODBUS_OK 成功，其他错误码
 */
int fujun_motor_read_status(uint16_t *status);

/**
 * @brief 检查电机是否正在运行（单次查询）
 * @return true=运行中, false=已停止/通讯失败
 */
bool fujun_motor_is_running(void);

/**
 * @brief 检查电机是否已停止（单次查询）
 * @return true=已停止, false=运行中/通讯失败
 */
bool fujun_motor_is_stopped(void);

/**
 * @brief 检查电机是否有报警（单次查询）
 * @return true=有报警, false=无报警/通讯失败
 */
bool fujun_motor_has_alarm(void);

/**
 * @brief 检查电机是否已到达目标位置
 * @return true=已到达, false=未到达/通讯失败
 */
bool fujun_motor_is_arrived(void);

/**
 * @brief 阻塞等待电机停止运行
 * @param timeout_ms 超时时间(ms), 建议 >= 30000
 * @return 0=已停止, -1=超时, -2=报警, -3=通讯失败
 */
int fujun_motor_wait_stop(uint32_t timeout_ms);

/**
 * @brief 移动电机到目标位置并等待到达（含重试）
 *
 * 算法：
 *   1. 读取当前位置，发送绝对位置移动指令
 *   2. 按 20000 pulses/s 换算等待时间 + 2s 余量，一次性等待
 *   3. 等待结束后读取位置，|实际 - 目标| <= tolerance 即判定到位
 *   4. 若不到位，自动重发移动指令重试，最多 3 次重试（共 4 次）
 *
 * @param target     目标位置 (pulses)
 * @return 0=已到达, -1=重试耗尽, -2=报警, -3=通讯失败
 *
 * 到位误差 FUJUN_POSITION_TOLERANCE，最大等待 FUJUN_WAIT_TIMEOUT_MS
 */
int fujun_motor_wait_position(int32_t target);

#endif
