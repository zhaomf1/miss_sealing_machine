#ifndef __DEV_CYLINDER_H
#define __DEV_CYLINDER_H

#include <stdint.h>
#include <stdbool.h>

/* 电缸ID：支持双电缸场景 */
typedef enum {
    CYLINDER_ID_SUCK = 0,       // 吸膜电缸
    CYLINDER_ID_SEAL = 1,       // 封膜电缸
} CylinderId_t;

typedef enum {
    CYLINDER_HOME_NOT_STARTED   = 0,    // 未回零
    CYLINDER_HOME_SUCCESS       = 1,    // 回零成功
    CYLINDER_HOME_IN_PROGRESS   = 2     // 回零中
} CylinderHomeState_t;

typedef enum {
    CYLINDER_MOTION_RUNNING     = 0,    // 运动中
    CYLINDER_MOTION_ARRIVED     = 1,    // 到达位置
    CYLINDER_MOTION_STALLED     = 2     // 堵转
} CylinderMotionState_t;

typedef enum {
    CYLINDER_JOG_BACKWARD   = -1,       // 反向点动
    CYLINDER_JOG_STOP       = 0,        // 停止点动
    CYLINDER_JOG_FORWARD    = 1         // 正向点动
} CylinderJogDir_t;

typedef enum {
    CYLINDER_HOME_DIR_FORWARD   = 0,    // 前进归零
    CYLINDER_HOME_DIR_BACKWARD  = 1     // 后退归零
} CylinderHomeDir_t;

typedef enum {
    CYLINDER_PUSH_DIR_FORWARD   = 0,    // 正向推压
    CYLINDER_PUSH_DIR_BACKWARD  = 1     // 反向推压
} CylinderPushDir_t;

int cylinder_home(CylinderId_t id);
int cylinder_set_force(CylinderId_t id, uint16_t force_percent);
int cylinder_set_push_length(CylinderId_t id, uint16_t length_001mm);
int cylinder_move_to(CylinderId_t id, uint16_t position_001mm);
int cylinder_set_max_speed(CylinderId_t id, uint16_t speed_percent);
int cylinder_set_accel(CylinderId_t id, uint16_t accel_percent);
int cylinder_move_relative(CylinderId_t id, int16_t offset_001mm);
int cylinder_jog(CylinderId_t id, CylinderJogDir_t dir);
int cylinder_read_home_state(CylinderId_t id, CylinderHomeState_t *state);
int cylinder_read_motion_state(CylinderId_t id, CylinderMotionState_t *state);
int cylinder_read_position(CylinderId_t id, uint16_t *position_001mm);
int cylinder_read_current(CylinderId_t id, uint16_t *current);
int cylinder_save_params(CylinderId_t id);
int cylinder_set_home_dir(CylinderId_t id, CylinderHomeDir_t dir);
int cylinder_set_push_speed(CylinderId_t id, uint16_t speed_percent);
int cylinder_set_push_dir(CylinderId_t id, CylinderPushDir_t dir);
int cylinder_io_test(CylinderId_t id, uint16_t group);
int cylinder_set_io_mode(CylinderId_t id, uint16_t enable);
int dev_cylinder_init(void);

/* ------------------------------------------------------------------ *
 *  动作到位检查 & 等待接口
 * ------------------------------------------------------------------ */

/**
 * @brief 检查电缸是否已到达目标位置（单次查询）
 * @param id 电缸 ID
 * @return true=已到位, false=运动中/堵转/通讯失败
 */
bool cylinder_is_arrived(CylinderId_t id);

/**
 * @brief 检查电缸是否仍在运动中（单次查询）
 * @param id 电缸 ID
 * @return true=运动中, false=已到位/堵转/通讯失败
 */
bool cylinder_is_moving(CylinderId_t id);

/**
 * @brief 检查电缸是否堵转（单次查询）
 * @param id 电缸 ID
 * @return true=堵转, false=正常/通讯失败
 */
bool cylinder_is_stalled(CylinderId_t id);

/**
 * @brief 检查电缸是否已回零成功（单次查询）
 * @param id 电缸 ID
 * @return true=回零成功, false=未回零/回零中/通讯失败
 */
bool cylinder_is_homed(CylinderId_t id);

/**
 * @brief 阻塞等待电缸运动到目标位置
 * @param id 电缸 ID
 * @param timeout_ms 超时时间(ms), 建议 >= 5000
 * @return 0=已到位, -1=超时, -2=堵转, -3=通讯失败
 */
int cylinder_wait_arrived(CylinderId_t id, uint32_t timeout_ms);

/**
 * @brief 阻塞等待电缸回零完成
 * @param id 电缸 ID
 * @param timeout_ms 超时时间(ms), 建议 >= 10000
 * @return 0=回零成功, -1=超时, -2=通讯失败
 */
int cylinder_wait_home(CylinderId_t id, uint32_t timeout_ms);

#endif
