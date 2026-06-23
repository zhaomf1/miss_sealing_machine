#ifndef __DEV_SUCTION_CUP_H
#define __DEV_SUCTION_CUP_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    SUCTION_CUP_RELEASE     = 0,    // 释放物体/初始化
    SUCTION_CUP_HOLDING     = 1,    // 吸住物体
    SUCTION_CUP_NOT_HOLD    = 2,    // 未吸住物体
    SUCTION_CUP_DROPPED     = 3     // 物体掉落
} SuctionCupState_t;

typedef enum {
    SUCTION_CMD_SUCK        = 1,    // 吸取
    SUCTION_CMD_RELEASE     = 2,    // 释放
    SUCTION_CMD_VACUUM_SUCK = 3     // 按真空度设置吸取
} SuctionCmd_t;

typedef enum {
    SUCTION_POINT_1 = 0,
    SUCTION_POINT_2 = 1,
    SUCTION_POINT_3 = 2,
    SUCTION_POINT_4 = 3
} SuctionPoint_t;

int suction_cup_suck(void);
int suction_cup_release(void);
int suction_cup_vacuum_suck(uint16_t min_vacuum, uint16_t max_vacuum);
int suction_cup_read_state(SuctionCupState_t *state);
int suction_cup_read_vacuum(uint16_t *vacuum);
int suction_cup_set_io_mode(uint16_t enable);
int suction_cup_set_judge_time(uint16_t seconds);
int suction_cup_set_point_config(SuctionPoint_t point, SuctionCmd_t cmd,
                                  uint16_t min_vacuum, uint16_t max_vacuum);
int suction_cup_save_params(void);
int suction_cup_read_device_info(uint16_t *model, uint16_t *hw_ver, uint16_t *sw_ver);
int dev_suction_cup_init(void);
int suction_cup_set_min_vacuum(uint16_t vacuum);
int suction_cup_set_max_vacuum(uint16_t vacuum);

/* ------------------------------------------------------------------ *
 *  动作到位检查 & 等待接口
 * ------------------------------------------------------------------ */

/**
 * @brief 检查吸盘当前状态（单次查询，不阻塞）
 * @param state 输出当前状态: 0=释放/初始化, 1=吸住物体, 2=未吸住, 3=掉落
 * @return true=读取成功, false=通讯失败
 */
bool suction_cup_check_state(SuctionCupState_t *state);

/**
 * @brief 检查吸盘是否已吸住物体
 * @return true=已吸住, false=未吸住或通讯失败
 */
bool suction_cup_is_holding(void);

/**
 * @brief 检查吸盘是否已释放
 * @return true=已释放, false=未释放或通讯失败
 */
bool suction_cup_is_released(void);

/**
 * @brief 阻塞等待吸盘达到期望状态
 * @param desired_state 期望的状态
 * @param timeout_ms 超时时间(ms), 建议 >= 2000
 * @return 0=达到期望状态, -1=超时, -2=通讯失败
 */
int suction_cup_wait_state(SuctionCupState_t desired_state, uint32_t timeout_ms);

/**
 * @brief 阻塞等待吸盘吸住物体
 * @param timeout_ms 超时时间(ms)
 * @return 0=吸住成功, -1=超时, -2=途中掉落/未吸住
 */
int suction_cup_wait_hold(uint32_t timeout_ms);

/**
 * @brief 阻塞等待吸盘完全释放
 * @param timeout_ms 超时时间(ms)
 * @return 0=释放成功, -1=超时, -2=通讯失败
 */
int suction_cup_wait_release(uint32_t timeout_ms);

#endif
