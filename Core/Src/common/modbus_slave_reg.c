#include <stdio.h>
#include "modbus_slave_reg.h"
#include "app_control.h"
#include "dev_cylinder.h"
#include "dev_suction_cup.h"
#include "dev_fujun_motor.h"
#include "cmsis_os2.h"


/* ===================================================================
 *                    寄存器映射表
 *
 *  按地址升序排列（用于二分查找）。
 *  新增寄存器：在此表添加一行 + 在读写分发开关中添加对应 case。
 * =================================================================== */

#define MODBUS_SLAVE_ADDR         0x01
#define MODBUS_MAX_REG_ADDR       0x00A1  // 最大寄存器地址 + 1
#define FIRMWARE_VERSION_CODE     101     // 固件版本号 (寄存器 0x0000)

/* modbus协议寄存器表 */
static const ModbusRegDef_t reg_map[] = {
    /* 2.1 系统命令 */
    {0x0000, REG_ACCESS_RO, REG_TYPE_U16},   // 获取版本信息
    {0x0001, REG_ACCESS_RW, REG_TYPE_U16},   // 修改设备地址
    {0x0002, REG_ACCESS_RO, REG_TYPE_U16},   // 获取设备故障信息

    /* 2.2 工作流程控制 */
    {0x0010, REG_ACCESS_WO, REG_TYPE_U16},   // 开始流程
    {0x0011, REG_ACCESS_WO, REG_TYPE_U16},   // 初始化流程
    {0x0012, REG_ACCESS_WO, REG_TYPE_U16},   // 结束流程
    {0x0013, REG_ACCESS_RO, REG_TYPE_U16},   // 获取系统状态

    /* 2.3 调试动作控制 */
    {0x0020, REG_ACCESS_RO, REG_TYPE_U16},   // 读取实时温度 (0.1°C单位)
    {0x0021, REG_ACCESS_RW, REG_TYPE_U16},   // 温控开关 (1=开启, 0=关闭)
    {0x0030, REG_ACCESS_WO, REG_TYPE_U32},   // 富俊电机移动距离
    {0x0040, REG_ACCESS_WO, REG_TYPE_U16},   // 吸膜电缸移动距离
    {0x0041, REG_ACCESS_WO, REG_TYPE_U16},   // 封膜电缸移动距离
    {0x0050, REG_ACCESS_WO, REG_TYPE_U16},   // 电动吸盘开关
    {0x0060, REG_ACCESS_WO, REG_TYPE_U16},   // 富俊电机归零
    {0x0061, REG_ACCESS_WO, REG_TYPE_U16},   // 吸膜动作
    {0x0062, REG_ACCESS_WO, REG_TYPE_U16},   // 铺膜动作
    {0x0063, REG_ACCESS_WO, REG_TYPE_U16},   // 封膜动作
    {0x0064, REG_ACCESS_WO, REG_TYPE_U16},   // 取/放孔板

    /* EEPROM 参数读写 */
    {0x0070, REG_ACCESS_RW, REG_TYPE_U32},   // 零点位置
    {0x0072, REG_ACCESS_RW, REG_TYPE_U32},   // 吸膜/封膜位置
    {0x0074, REG_ACCESS_RW, REG_TYPE_U32},   // 铺膜点位置
    {0x0076, REG_ACCESS_RW, REG_TYPE_U32},   // 取放孔板位置
    {0x0078, REG_ACCESS_RW, REG_TYPE_U16},   // 温度值
    {0x0079, REG_ACCESS_RW, REG_TYPE_U16},   // 压膜时间
    {0x0080, REG_ACCESS_RO, REG_TYPE_U32},   // 总封膜次数

    /* 调试动作执行状态 */
    {0x00A0, REG_ACCESS_RO, REG_TYPE_U16},   // 调试动作执行状态
};

#define REG_COUNT  (sizeof(reg_map) / sizeof(reg_map[0]))

/* ---- 全局状态 ---- */
static uint16_t shadow[MODBUS_MAX_REG_ADDR];    // 寄存器影子缓存
static uint8_t  g_slave_addr = MODBUS_SLAVE_ADDR;
static uint8_t  g_pending_slave_addr = MODBUS_SLAVE_ADDR;
static uint8_t  g_slave_addr_pending = 0;

/* ===================================================================
 *          调试动作执行状态追踪 (寄存器 0x00A0)
 *
 *  0x00A0 = (来源地址 << 8) | 结果码
 *  结果码: 0x00=成功, 0x01=失败, 0x02=执行中, 0xFF=无历史
 * =================================================================== */

#define ACT_RESULT_SUCCESS     0x00
#define ACT_RESULT_FAILURE     0x01
#define ACT_RESULT_EXECUTING   0x02
#define ACT_RESULT_NONE        0xFF

static uint16_t g_action_status = 0xFFFF;

/* ---- 故障位图 (寄存器 0x0002) ---- */
#define FAULT_BIT_TEMP           (1 << 0)
#define FAULT_BIT_SUCK_CYL       (1 << 1)
#define FAULT_BIT_SEAL_CYL       (1 << 2)
#define FAULT_BIT_LEAD_SCREW     (1 << 3)
#define FAULT_BIT_SUCTION_CUP    (1 << 4)

static volatile uint16_t g_fault_status = 0x0000;

/* ---- 系统工作状态 (0x0013) ---- */
static volatile uint16_t g_system_state = SYS_STATE_UNINITIALIZED;
static volatile uint8_t g_stop_requested = 0;

static void cmd_stop_workflow(void);

uint16_t modbus_reg_get_system_state(void)
{
    return g_system_state;
}

void modbus_reg_set_system_state(uint16_t state)
{
    g_system_state = state;
    printf("[SYS] State: %s\r\n",
           state == SYS_STATE_UNINITIALIZED    ? "UNINITIALIZED" :
           state == SYS_STATE_INITIALIZING     ? "INITIALIZING" :
           state == SYS_STATE_INITIALIZED      ? "INITIALIZED" :
           state == SYS_STATE_RUNNING          ? "RUNNING" :
           state == SYS_STATE_FINISHED         ? "FINISHED" :
           state == SYS_STATE_WORKFLOW_FAILED  ? "WORKFLOW_FAILED" : "UNKNOWN");
}

uint8_t modbus_reg_is_stop_requested(void)
{
    return g_stop_requested;
}

void modbus_reg_set_temp_fault(uint8_t fault)
{
    if (fault) {
        g_fault_status |= FAULT_BIT_TEMP;
    } else {
        g_fault_status &= (uint16_t)~FAULT_BIT_TEMP;
    }
}

/* ---- 调试动作超时 (ms) ---- */
#define TIMEOUT_CYL_MOVE      10000
#define TIMEOUT_SUCTION_CUP    5000
#define SUCTION_CYL_DOWN_WAIT_MS       1500

/* ---- 复合动作重试次数 ---- */
#define COMPOUND_ACTION_MAX_RETRIES  3

/* ---- 复合动作电缸下落距离 (0.01mm) ---- */
#define SUCTION_CYL_SUCK_POS   5200  // 吸膜点：吸膜操作时电缸下落距离 
#define SUCTION_CYL_PAVE_POS   1500  // 铺膜点：铺膜操作时电缸下落距离 
#define SEAL_CYL_DROP_DISTANCE  5000  // 封膜电缸下落距离 

/* ---- EEPROM 参数全局缓存（上电初始化一次，避免频繁操作 EEPROM）---- */
static uint32_t g_eeprom_zero;       // 零点位置 (0x0070)
static uint32_t g_eeprom_suck_seal;  // 吸膜/封膜位置 (0x0072)
static uint32_t g_eeprom_pave;       // 铺膜点位置 (0x0074)
static uint32_t g_eeprom_get_place;  // 取放孔板位置 (0x0076)
static uint16_t g_eeprom_temp_ctrl;  // 温度值 (0x0078)
static uint16_t g_eeprom_press_time; // 压膜时间 (0x0079)
static uint32_t g_eeprom_frequency;  // 总封膜次数 (0x18)


/* ===================================================================
 *              内部辅助
 * =================================================================== */

static const char *eeprom_param_name(uint16_t reg_addr)
{
    switch (reg_addr) {
        case 0x0070: return "zero_pos";
        case 0x0072: return "suck_seal_pos";
        case 0x0074: return "pave_pos";
        case 0x0076: return "get_place_pos";
        case 0x0078: return "temperature_setpoint";
        case 0x0079: return "press_time";
        case 0x0080: return "seal_count";
        default:     return "unknown";
    }
}

static void action_set_result(uint8_t reg_addr, uint8_t result)
{
    g_action_status = ((uint16_t)reg_addr << 8) | result;
}

/**
 * @brief 根据调试动作结果更新故障位图 0x0002
 */
static void fault_update(uint8_t reg_addr, uint8_t result)
{
    if (result == ACT_RESULT_SUCCESS) {
        switch (reg_addr) {
            case 0x40: g_fault_status &= ~FAULT_BIT_SUCK_CYL;    break;
            case 0x41: g_fault_status &= ~FAULT_BIT_SEAL_CYL;    break;
            case 0x30: g_fault_status &= ~FAULT_BIT_LEAD_SCREW;  break;
            case 0x50: g_fault_status &= ~FAULT_BIT_SUCTION_CUP; break;
        }
    } else {
        switch (reg_addr) {
            case 0x40: g_fault_status |= FAULT_BIT_SUCK_CYL;    break;
            case 0x41: g_fault_status |= FAULT_BIT_SEAL_CYL;    break;
            case 0x30: g_fault_status |= FAULT_BIT_LEAD_SCREW;  break;
            case 0x50: g_fault_status |= FAULT_BIT_SUCTION_CUP; break;
        }
    }
}


/* ===================================================================
 *              温度控制预留桩
 * =================================================================== */

/** @brief  读取当前温度，单位 0.1°C */
static int temp_ctrl_get(uint16_t *temperature)
{
    if (temperature == NULL) return -1;
    float temp = PID_GetCurrentTemperature();
    if (temp < 0.0f) temp = 0.0f;
    if (temp > 250.0f) temp = 250.0f;
    *temperature = (uint16_t)(temp * 10.0f);   /* 温度值 ×10，单位 0.1°C */
    return 0;
}

static int delay_interruptible(uint32_t delay_ms)
{
    uint32_t elapsed = 0;
    while (elapsed < delay_ms) {
        uint32_t slice = (delay_ms - elapsed > 50U) ? 50U : (delay_ms - elapsed);
        if (g_stop_requested) return ACTION_WAIT_CANCELLED;
        osDelay(slice);
        elapsed += slice;
    }
    return 0;
}

/* 吸膜下压以电缸运动状态判定，不比较实际位置。 */
static int suck_cyl_move_down_wait_stop(uint16_t position)
{
    CylinderMotionState_t state;
    int ret = cylinder_move_to(CYLINDER_ID_SUCK, position);

    if (ret != 0) {
        printf("[CMD_0x0061] Step2 Suck cyl move cmd FAIL target=%u ret=%d\r\n",
               position, ret);
        return -3;
    }

    if (delay_interruptible(SUCTION_CYL_DOWN_WAIT_MS) != 0) {
        return ACTION_WAIT_CANCELLED;
    }

    ret = cylinder_read_motion_state(CYLINDER_ID_SUCK, &state);
    if (ret != 0) {
        printf("[CMD_0x0061] Step2 Suck cyl state read FAIL ret=%d\r\n", ret);
        return -3;
    }

    if (state == CYLINDER_MOTION_ARRIVED || state == CYLINDER_MOTION_STALLED) {
        printf("[CMD_0x0061] Step2 Suck cyl down complete target=%u state=%d (%ums)\r\n",
               position, (int)state, SUCTION_CYL_DOWN_WAIT_MS);
        return 0;
    }

    if (state == CYLINDER_MOTION_RUNNING) {
        printf("[CMD_0x0061] Step2 Suck cyl still running target=%u (%ums)\r\n",
               position, SUCTION_CYL_DOWN_WAIT_MS);
    } else {
        printf("[CMD_0x0061] Step2 Suck cyl invalid state=%d\r\n", (int)state);
    }
    return -1;
}

static int suck_cyl_return_to_zero(void)
{
    int ret = -1;

    for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
        ret = cylinder_move_to_wait(CYLINDER_ID_SUCK, 0, TIMEOUT_CYL_MOVE);
        if (ret == 0 || ret == ACTION_WAIT_CANCELLED) {
            break;
        }
        printf("[CMD_0x0061] Cleanup return attempt %d/%d FAIL (ret=%d)\r\n",
               attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
    }

    return ret;
}


/* ===================================================================
 *          复合动作实现 (0x0060~0x0064)
 *
 *  每个复合动作包含多个子步骤，任一步骤失败则记录失败状态
 *  并设置对应外设的故障位，后续步骤不再执行。
 * =================================================================== */

/**
 * @brief 0x0060 — 放膜动作
 * 读取 EEPROM 零点位置 (0x0070)，控制富俊电机移动到该位置。
 */
static void cmd_fujun_home(uint16_t value)
{
    (void)value;
    uint8_t result = ACT_RESULT_FAILURE;
    int ret = -1;

    printf("[CMD_0x0060] Fujun motor home: zero_pos=%lu\r\n",
           (unsigned long)g_eeprom_zero);

    for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
        ret = fujun_motor_wait_position((int32_t)g_eeprom_zero);
        if (ret == 0) break;
        if (ret == ACTION_WAIT_CANCELLED) break;
        printf("[CMD_0x0060] Attempt %d/%d FAIL (ret=%d)\r\n",
               attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
    }

    if (ret == 0) {
        result = ACT_RESULT_SUCCESS;
        g_fault_status &= ~FAULT_BIT_LEAD_SCREW;
        printf("[CMD_0x0060] Fujun motor home OK\r\n");
    } else {
        g_fault_status |= FAULT_BIT_LEAD_SCREW;
        printf("[CMD_0x0060] Fujun motor home FAIL after %d retries (ret=%d)\r\n",
               COMPOUND_ACTION_MAX_RETRIES, ret);
    }

    action_set_result(0x60, result);
}

/**
 * @brief 0x0061 — 吸膜动作
 * 流程：富俊电机移动到吸膜/封膜点 → 吸膜电缸下落 → 吸盘吸取 → 吸膜电缸归零
 */
static void cmd_suction_action(uint16_t value)
{
    (void)value;
    uint8_t result = ACT_RESULT_FAILURE;
    uint8_t suck_cyl_commanded = 0;
    int ret;

    printf("[CMD_0x0061] Suction action: fujun_target=%lu\r\n",
           (unsigned long)g_eeprom_suck_seal);

    /* Step 1: 富俊电机移动到吸膜/封膜位置 */
    ret = -1;
    for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
        ret = fujun_motor_wait_position((int32_t)g_eeprom_suck_seal);
        if (ret == 0) break;
        if (ret == ACTION_WAIT_CANCELLED) break;
        printf("[CMD_0x0061] Step1 attempt %d/%d FAIL (ret=%d)\r\n",
               attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
    }
    if (ret != 0) {
        g_fault_status |= FAULT_BIT_LEAD_SCREW;
        printf("[CMD_0x0061] Step1 Fujun move FAIL after %d retries\r\n", COMPOUND_ACTION_MAX_RETRIES);
        goto done;
    }
    g_fault_status &= ~FAULT_BIT_LEAD_SCREW;

    /* Step 2: 吸膜电缸下落到吸膜点 */
    ret = -1;
    for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
        suck_cyl_commanded = 1;
        ret = suck_cyl_move_down_wait_stop(SUCTION_CYL_SUCK_POS);
        if (ret == 0) break;
        if (ret == ACTION_WAIT_CANCELLED) break;
        printf("[CMD_0x0061] Step2 attempt %d/%d FAIL (ret=%d)\r\n",
               attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
    }
    if (ret != 0) {
        g_fault_status |= FAULT_BIT_SUCK_CYL;
        printf("[CMD_0x0061] Step2 Suck cyl drop FAIL after %d retries\r\n", COMPOUND_ACTION_MAX_RETRIES);
        goto done;
    }
    g_fault_status &= ~FAULT_BIT_SUCK_CYL;

    /* Step 3: 吸盘开启，等待吸住 */
    ret = -1;
    for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
        /* 重试前先释放吸盘，确保硬件状态机从上次失败中复位 */
        if (attempt > 0) {
            suction_cup_release();
            if (delay_interruptible(500) != 0) goto done;
        }
        suction_cup_suck();
        ret = suction_cup_wait_hold(TIMEOUT_SUCTION_CUP);
        if (ret == 0) break;
        if (ret == ACTION_WAIT_CANCELLED) break;
        printf("[CMD_0x0061] Step3 attempt %d/%d FAIL (ret=%d)\r\n",
               attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
    }
    if (ret != 0) {
        g_fault_status |= FAULT_BIT_SUCTION_CUP;
        printf("[CMD_0x0061] Step3 Suction cup hold FAIL after %d retries\r\n", COMPOUND_ACTION_MAX_RETRIES);
        goto done;
    }
    g_fault_status &= ~FAULT_BIT_SUCTION_CUP;

    /* 吸盘吸取成功后等待 1s 确保吸稳 */
    if (delay_interruptible(1000) != 0) goto done;

    /* Step 4: 吸膜电缸移动到0 */
    ret = suck_cyl_return_to_zero();
    if (ret != 0) {
        g_fault_status |= FAULT_BIT_SUCK_CYL;
        printf("[CMD_0x0061] Step4 Suck cyl move to 0 FAIL after %d retries\r\n", COMPOUND_ACTION_MAX_RETRIES);
        goto done;
    }
    suck_cyl_commanded = 0;
    g_fault_status &= ~FAULT_BIT_SUCK_CYL;

    result = ACT_RESULT_SUCCESS;

done:
    if (result != ACT_RESULT_SUCCESS && suck_cyl_commanded) {
        printf("[CMD_0x0061] Cleanup: release suction cup and return suction cylinder to 0\r\n");
        if (suction_cup_release() != 0 ||
            suction_cup_wait_release(TIMEOUT_SUCTION_CUP) != 0) {
            g_fault_status |= FAULT_BIT_SUCTION_CUP;
            printf("[CMD_0x0061] Cleanup: suction cup release FAIL\r\n");
        }
        if (suck_cyl_return_to_zero() != 0) {
            g_fault_status |= FAULT_BIT_SUCK_CYL;
            printf("[CMD_0x0061] Cleanup: suction cylinder return to 0 FAIL\r\n");
        }
    }
    action_set_result(0x61, result);
    printf("[CMD_0x0061] Suction action %s\r\n",
           (result == ACT_RESULT_SUCCESS) ? "OK" : "FAIL");
}

/**
 * @brief 0x0062 — 铺膜动作
 * 流程：富俊电机移动到铺膜点 → 吸膜电缸下落 → 吸盘释放 → 吸膜电缸归零
 */
static void cmd_lay_film_action(uint16_t value)
{
    (void)value;
    uint8_t result = ACT_RESULT_FAILURE;
    int ret;

    printf("[CMD_0x0062] Lay film action: fujun_target=%lu\r\n",
           (unsigned long)g_eeprom_pave);

    /* Step 1: 富俊电机移动到铺膜位置 */
    ret = -1;
    for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
        ret = fujun_motor_wait_position((int32_t)g_eeprom_pave);
        if (ret == 0) break;
        if (ret == ACTION_WAIT_CANCELLED) break;
        printf("[CMD_0x0062] Step1 attempt %d/%d FAIL (ret=%d)\r\n",
               attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
    }
    if (ret != 0) {
        g_fault_status |= FAULT_BIT_LEAD_SCREW;
        printf("[CMD_0x0062] Step1 Fujun move FAIL after %d retries\r\n", COMPOUND_ACTION_MAX_RETRIES);
        goto done;
    }
    g_fault_status &= ~FAULT_BIT_LEAD_SCREW;

    /* Step 2: 吸膜电缸下落到铺膜点 */
    ret = -1;
    for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
        ret = cylinder_move_to_wait(CYLINDER_ID_SUCK, SUCTION_CYL_PAVE_POS, TIMEOUT_CYL_MOVE);
        if (ret == 0) break;
        if (ret == ACTION_WAIT_CANCELLED) break;
        printf("[CMD_0x0062] Step2 attempt %d/%d FAIL (ret=%d)\r\n",
               attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
    }
    if (ret != 0) {
        g_fault_status |= FAULT_BIT_SUCK_CYL;
        printf("[CMD_0x0062] Step2 Suck cyl drop FAIL after %d retries\r\n", COMPOUND_ACTION_MAX_RETRIES);
        goto done;
    }
    g_fault_status &= ~FAULT_BIT_SUCK_CYL;

    /* Step 3: 吸盘释放，等待完全释放 */
    ret = -1;
    for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
        /* 重试前先吸取再释放，确保硬件状态机复位 */
        if (attempt > 0) {
            if (suction_cup_suck() != 0) {
                printf("[CMD_0x0062] Step3 retry suck cmd FAIL, skip wait\r\n");
                continue;  // 通讯失败，跳过本轮等待，进入下一次重试
            }
            if (delay_interruptible(500) != 0) goto done;
        }
        if (suction_cup_release() != 0) {
            printf("[CMD_0x0062] Step3 release cmd FAIL, skip wait\r\n");
            continue;  // 释放命令未送达，跳过轮询，进入下一次重试
        }
        ret = suction_cup_wait_release(TIMEOUT_SUCTION_CUP);
        if (ret == 0) break;
        if (ret == ACTION_WAIT_CANCELLED) break;
        printf("[CMD_0x0062] Step3 attempt %d/%d FAIL (ret=%d)\r\n",
               attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
    }
    if (ret != 0) {
        g_fault_status |= FAULT_BIT_SUCTION_CUP;
        printf("[CMD_0x0062] Step3 Suction cup release FAIL after %d retries\r\n", COMPOUND_ACTION_MAX_RETRIES);
        goto done;
    }
    g_fault_status &= ~FAULT_BIT_SUCTION_CUP;

    /* 释放成功后额外等待，确保膜材靠重力完全脱离吸盘 */
    if (delay_interruptible(1000) != 0) goto done;
    printf("[CMD_0x0062] Step3 Suction cup release OK\r\n");

    /* Step 4: 吸膜电缸移动到0 */
    ret = -1;
    for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
        ret = cylinder_move_to_wait(CYLINDER_ID_SUCK, 0, TIMEOUT_CYL_MOVE);
        if (ret == 0) break;
        if (ret == ACTION_WAIT_CANCELLED) break;
        printf("[CMD_0x0062] Step4 attempt %d/%d FAIL (ret=%d)\r\n",
               attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
    }
    if (ret != 0) {
        g_fault_status |= FAULT_BIT_SUCK_CYL;
        printf("[CMD_0x0062] Step4 Suck cyl move to 0 FAIL after %d retries\r\n", COMPOUND_ACTION_MAX_RETRIES);
        goto done;
    }
    g_fault_status &= ~FAULT_BIT_SUCK_CYL;

    result = ACT_RESULT_SUCCESS;

done:
    action_set_result(0x62, result);
    printf("[CMD_0x0062] Lay film action %s\r\n",
           (result == ACT_RESULT_SUCCESS) ? "OK" : "FAIL");
}

/**
 * @brief 0x0063 — 封膜动作
 * 流程：富俊电机移动到吸膜/封膜点 → 封膜电缸下落 → 保压等待 → 封膜电缸归零
 */
static void cmd_seal_action(uint16_t value)
{
    (void)value;
    uint8_t result = ACT_RESULT_FAILURE;
    int ret;

    printf("[CMD_0x0063] Seal action: fujun_target=%lu press_time=%ums\r\n",
           (unsigned long)g_eeprom_suck_seal, g_eeprom_press_time);

    /* Step 1: 富俊电机移动到吸膜/封膜位置 */
    ret = -1;
    for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
        ret = fujun_motor_wait_position((int32_t)g_eeprom_suck_seal);
        if (ret == 0) break;
        if (ret == ACTION_WAIT_CANCELLED) break;
        printf("[CMD_0x0063] Step1 attempt %d/%d FAIL (ret=%d)\r\n",
               attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
    }
    if (ret != 0) {
        g_fault_status |= FAULT_BIT_LEAD_SCREW;
        printf("[CMD_0x0063] Step1 Fujun move FAIL after %d retries\r\n", COMPOUND_ACTION_MAX_RETRIES);
        goto done;
    }
    g_fault_status &= ~FAULT_BIT_LEAD_SCREW;

    /* Step 2: 封膜电缸下落 */
    ret = -1;
    for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
        ret = cylinder_move_to_wait(CYLINDER_ID_SEAL, SEAL_CYL_DROP_DISTANCE, TIMEOUT_CYL_MOVE);
        if (ret == 0) break;
        if (ret == ACTION_WAIT_CANCELLED) break;
        printf("[CMD_0x0063] Step2 attempt %d/%d FAIL (ret=%d)\r\n",
               attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
    }
    if (ret != 0) {
        g_fault_status |= FAULT_BIT_SEAL_CYL;
        printf("[CMD_0x0063] Step2 Seal cyl drop FAIL after %d retries\r\n", COMPOUND_ACTION_MAX_RETRIES);
        goto done;
    }
    g_fault_status &= ~FAULT_BIT_SEAL_CYL;

    /* Step 3: 保压等待（读取压膜时间寄存器值），上限 60s 防溢出 */
    if (g_eeprom_press_time > 0) {
        uint32_t hold_ms = (uint32_t)g_eeprom_press_time * 1000;
        if (hold_ms > 60000) hold_ms = 60000;
        printf("[CMD_0x0063] Step3 Holding for %lums...\r\n", (unsigned long)hold_ms);
        if (delay_interruptible(hold_ms) != 0) goto done;
    }

    /* Step 4: 封膜电缸移动到0 */
    ret = -1;
    for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
        ret = cylinder_move_to_wait(CYLINDER_ID_SEAL, 0, TIMEOUT_CYL_MOVE);
        if (ret == 0) break;
        if (ret == ACTION_WAIT_CANCELLED) break;
        printf("[CMD_0x0063] Step4 attempt %d/%d FAIL (ret=%d)\r\n",
               attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
    }
    if (ret != 0) {
        g_fault_status |= FAULT_BIT_SEAL_CYL;
        printf("[CMD_0x0063] Step4 Seal cyl move to 0 FAIL after %d retries\r\n", COMPOUND_ACTION_MAX_RETRIES);
        goto done;
    }
    g_fault_status &= ~FAULT_BIT_SEAL_CYL;

    result = ACT_RESULT_SUCCESS;

done:
    action_set_result(0x63, result);
    printf("[CMD_0x0063] Seal action %s\r\n",
           (result == ACT_RESULT_SUCCESS) ? "OK" : "FAIL");
}

/**
 * @brief 0x0064 — 取/放孔板
 * 读取 EEPROM 取放孔板位置 (0x0076)，控制富俊电机移动到该位置。
 */
static void cmd_well_plate_action(uint16_t value)
{
    (void)value;
    uint8_t result = ACT_RESULT_FAILURE;
    int ret = -1;

    printf("[CMD_0x0064] Well plate: fujun_target=%lu\r\n",
           (unsigned long)g_eeprom_get_place);

    for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
        ret = fujun_motor_wait_position((int32_t)g_eeprom_get_place);
        if (ret == 0) break;
        if (ret == ACTION_WAIT_CANCELLED) break;
        printf("[CMD_0x0064] Attempt %d/%d FAIL (ret=%d)\r\n",
               attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
    }

    if (ret == 0) {
        result = ACT_RESULT_SUCCESS;
        g_fault_status &= ~FAULT_BIT_LEAD_SCREW;
        printf("[CMD_0x0064] Well plate OK\r\n");
    } else {
        g_fault_status |= FAULT_BIT_LEAD_SCREW;
        printf("[CMD_0x0064] Well plate FAIL after %d retries (ret=%d)\r\n",
               COMPOUND_ACTION_MAX_RETRIES, ret);
    }

    action_set_result(0x64, result);
}

/**
 * @brief 停止流程
 * 温控停止 → 封膜电缸归零 → 吸盘关闭 → 吸膜电缸归零 → 富俊电机归零
 */
static void cmd_stop_workflow(void)
{
    uint8_t result = ACT_RESULT_SUCCESS;
    uint16_t prev_state = g_system_state;
    int ret;

    /* 停止恢复动作自身不应再被取消 */
    g_stop_requested = 0;

    /* 未初始化状态下外设不可用，无需执行停止操作 */
    if (g_system_state == SYS_STATE_UNINITIALIZED) {
        printf("[STOP] System not initialized, skip\r\n");
        action_set_result(0x12, ACT_RESULT_SUCCESS);
        modbus_reg_set_system_state(SYS_STATE_FINISHED);
        return;
    }

    printf("[STOP] === Stop workflow ===\r\n");

    /* Step 1: 温控模块停止加热 */
    printf("[STOP] Step1: Stop heating\r\n");
    temp_ctrl_stop();

    /* Step 2: 封膜电缸归零 */
    printf("[STOP] Step2: Seal cylinder home\r\n");
    ret = cylinder_home(CYLINDER_ID_SEAL);
    if (ret != 0) {
        printf("[STOP] Step2: Seal cylinder home cmd FAIL (ret=%d)\r\n", ret);
        g_fault_status |= FAULT_BIT_SEAL_CYL;
        result = ACT_RESULT_FAILURE;
    } else if (cylinder_wait_home(CYLINDER_ID_SEAL, 30000) != 0) {
        printf("[STOP] Step2: Seal cylinder home FAIL\r\n");
        g_fault_status |= FAULT_BIT_SEAL_CYL;
        result = ACT_RESULT_FAILURE;
    } else {
        g_fault_status &= (uint16_t)~FAULT_BIT_SEAL_CYL;
        printf("[STOP] Step2: Seal cylinder home OK\r\n");
    }

    /* Step 3: 电动吸盘关闭 (释放) */
    printf("[STOP] Step3: Release suction cup\r\n");
    if (suction_cup_release() != 0 ||
        suction_cup_wait_release(TIMEOUT_SUCTION_CUP) != 0) {
        g_fault_status |= FAULT_BIT_SUCTION_CUP;
        result = ACT_RESULT_FAILURE;
        printf("[STOP] Step3: Suction cup release FAIL\r\n");
    } else {
        g_fault_status &= (uint16_t)~FAULT_BIT_SUCTION_CUP;
        printf("[STOP] Step3: Suction cup released\r\n");
    }

    /* Step 4: 吸膜电缸归零 */
    printf("[STOP] Step4: Suction cylinder home\r\n");
    ret = cylinder_home(CYLINDER_ID_SUCK);
    if (ret != 0) {
        printf("[STOP] Step4: Suction cylinder home cmd FAIL (ret=%d)\r\n", ret);
        g_fault_status |= FAULT_BIT_SUCK_CYL;
        result = ACT_RESULT_FAILURE;
    } else if (cylinder_wait_home(CYLINDER_ID_SUCK, 30000) != 0) {
        printf("[STOP] Step4: Suction cylinder home FAIL\r\n");
        g_fault_status |= FAULT_BIT_SUCK_CYL;
        result = ACT_RESULT_FAILURE;
    } else {
        g_fault_status &= (uint16_t)~FAULT_BIT_SUCK_CYL;
        printf("[STOP] Step4: Suction cylinder home OK\r\n");
    }

    /* Step 5: 富俊电机归零 */
    printf("[STOP] Step5: Fujun motor home\r\n");
    ret = fujun_motor_wait_position((int32_t)g_eeprom_zero);
    if (ret == 0) {
        g_fault_status &= ~FAULT_BIT_LEAD_SCREW;
        printf("[STOP] Step5: Fujun motor home OK\r\n");
    } else {
        g_fault_status |= FAULT_BIT_LEAD_SCREW;
        printf("[STOP] Step5: Fujun motor home FAIL (ret=%d)\r\n", ret);
        result = ACT_RESULT_FAILURE;
    }

    action_set_result(0x12, result);

    if (result != ACT_RESULT_SUCCESS || prev_state == SYS_STATE_WORKFLOW_FAILED) {
        modbus_reg_set_system_state(SYS_STATE_WORKFLOW_FAILED);
    } else {
        modbus_reg_set_system_state(SYS_STATE_FINISHED);
    }

    printf("[STOP] === Stop workflow %s ===\r\n",
           (result == ACT_RESULT_SUCCESS) ? "complete" : "FAIL");
}

/**
 * @brief 0x0011 — 初始化流程
 * 温控 → 吸盘释放 → 双电缸归零 → 富俊电机归零
 */
static void cmd_init_workflow(void)
{
    uint8_t result = ACT_RESULT_SUCCESS;  // 默认成功，出错时改为 FAILURE

    g_stop_requested = 0;
    printf("[INIT] === Init sequence started ===\r\n");

    /* 状态切换: 未初始化 → 初始化中 */
    modbus_reg_set_system_state(SYS_STATE_INITIALIZING);

    /* ---- Step 1: 温控开启 ---- */
    printf("[INIT] Step1: Temperature to 180°C (placeholder)\r\n");
    temp_ctrl_start();

    /* ---- Step 2: 电动吸盘初始化 ---- */
    printf("[INIT] Step2: Initializing suction cup...\r\n");
    if (dev_suction_cup_init() != 0) {
        printf("[INIT] Step2: Suction cup init FAIL\r\n");
        g_fault_status |= FAULT_BIT_SUCTION_CUP;
        result = ACT_RESULT_FAILURE;
        goto done;
    }
    g_fault_status &= (uint16_t)~FAULT_BIT_SUCTION_CUP;
    printf("[INIT] Step2: Suction cup init OK\r\n");

    /* ---- Step 3: 吸膜电缸 + 封膜电缸归零 ---- */
    printf("[INIT] Step3: Initializing cylinders...\r\n");
    if (dev_cylinder_init() != 0) {
        printf("[INIT] Step3: Cylinders init FAIL\r\n");
        g_fault_status |= (FAULT_BIT_SUCK_CYL | FAULT_BIT_SEAL_CYL);
        result = ACT_RESULT_FAILURE;
        goto done;
    }
    g_fault_status &= (uint16_t)~(FAULT_BIT_SUCK_CYL | FAULT_BIT_SEAL_CYL);
    printf("[INIT] Step3: Cylinders init OK\r\n");

    /* ---- Step 4: 富俊电机初始化 ---- */
    printf("[INIT] Step4: Initializing Fujun motor...\r\n");
    if (dev_fujun_motor_init() == 0) {
        g_fault_status &= (uint16_t)~FAULT_BIT_LEAD_SCREW;
        printf("[INIT] Step4: Fujun motor init OK\r\n");
    } else {
        printf("[INIT] Step4: Fujun motor init FAIL\r\n");
        g_fault_status |= FAULT_BIT_LEAD_SCREW;
        result = ACT_RESULT_FAILURE;
        goto done;
    }

done:
    if (g_stop_requested) {
        printf("[INIT] Stop requested during initialization\r\n");
        cmd_stop_workflow();
        return;
    }

    /* 状态切换 */
    if (result == ACT_RESULT_SUCCESS) {
        modbus_reg_set_system_state(SYS_STATE_INITIALIZED);
    } else {
        modbus_reg_set_system_state(SYS_STATE_WORKFLOW_FAILED);
    }

    action_set_result(0x11, result);
    printf("[INIT] === Init sequence %s ===\r\n",
           (result == ACT_RESULT_SUCCESS) ? "completed" : "FAIL");
}

/**
 * @brief 0x0030 — 富俊电机移动到指定位置
 */
static void cmd_fujun_move_to(int32_t target)
{
    uint8_t r = ACT_RESULT_FAILURE;
    int ret = -1;
    for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
        ret = fujun_motor_wait_position(target);
        if (ret == 0) break;
        if (ret == ACTION_WAIT_CANCELLED) break;
        printf("[CMD_0x0030] Attempt %d/%d FAIL (ret=%d)\r\n",
               attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
    }
    if (ret == 0) {
        r = ACT_RESULT_SUCCESS;
        g_fault_status &= ~FAULT_BIT_LEAD_SCREW;
        printf("[CMD_0x0030] Fujun move to %ld OK\r\n", (long)target);
    } else {
        g_fault_status |= FAULT_BIT_LEAD_SCREW;
        printf("[CMD_0x0030] Fujun move to %ld FAIL after %d retries\r\n",
               (long)target, COMPOUND_ACTION_MAX_RETRIES);
    }
    action_set_result(0x30, r);
}

/**
 * @brief 0x0040 — 吸膜电缸移动到指定位置
 */
static void cmd_suck_cyl_move(uint16_t position)
{
    uint8_t r = ACT_RESULT_FAILURE;
    int ret = -1;
    for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
        ret = cylinder_move_to_wait(CYLINDER_ID_SUCK, position, TIMEOUT_CYL_MOVE);
        if (ret == 0) break;
        if (ret == ACTION_WAIT_CANCELLED) break;
        printf("[CMD_0x0040] Attempt %d/%d FAIL (ret=%d)\r\n",
               attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
    }
    if (ret == 0) {
        r = ACT_RESULT_SUCCESS;
        g_fault_status &= ~FAULT_BIT_SUCK_CYL;
    } else {
        g_fault_status |= FAULT_BIT_SUCK_CYL;
    }
    action_set_result(0x40, r);
    printf("[CMD_0x0040] Suck cyl move to %u: %s\r\n",
           position, (r == ACT_RESULT_SUCCESS) ? "OK" : "FAIL");
}

/**
 * @brief 0x0041 — 封膜电缸移动到指定位置
 */
static void cmd_seal_cyl_move(uint16_t position)
{
    uint8_t r = ACT_RESULT_FAILURE;
    int ret = -1;
    for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
        ret = cylinder_move_to_wait(CYLINDER_ID_SEAL, position, TIMEOUT_CYL_MOVE);
        if (ret == 0) break;
        if (ret == ACTION_WAIT_CANCELLED) break;
        printf("[CMD_0x0041] Attempt %d/%d FAIL (ret=%d)\r\n",
               attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
    }
    if (ret == 0) {
        r = ACT_RESULT_SUCCESS;
        g_fault_status &= ~FAULT_BIT_SEAL_CYL;
    } else {
        g_fault_status |= FAULT_BIT_SEAL_CYL;
    }
    action_set_result(0x41, r);
    printf("[CMD_0x0041] Seal cyl move to %u: %s\r\n",
           position, (r == ACT_RESULT_SUCCESS) ? "OK" : "FAIL");
}

/**
 * @brief 0x0050 — 电动吸盘吸取/释放
 */
static void cmd_suction_cup_ctrl(uint16_t op)
{
    uint8_t r = ACT_RESULT_FAILURE;
    int ret = -1;
    if (op == 1) {
        for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
            if (attempt > 0) { suction_cup_release(); osDelay(500); }
            suction_cup_suck();
            ret = suction_cup_wait_hold(TIMEOUT_SUCTION_CUP);
            if (ret == 0) break;
            if (ret == ACTION_WAIT_CANCELLED) break;
            printf("[CMD_0x0050] Hold attempt %d/%d FAIL (ret=%d)\r\n",
                   attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
        }
    } else {
        for (int attempt = 0; attempt < COMPOUND_ACTION_MAX_RETRIES; attempt++) {
            if (attempt > 0) { suction_cup_suck(); osDelay(500); }
            suction_cup_release();
            ret = suction_cup_wait_release(TIMEOUT_SUCTION_CUP);
            if (ret == 0) break;
            if (ret == ACTION_WAIT_CANCELLED) break;
            printf("[CMD_0x0050] Release attempt %d/%d FAIL (ret=%d)\r\n",
                   attempt + 1, COMPOUND_ACTION_MAX_RETRIES, ret);
        }
    }
    r = (ret == 0) ? ACT_RESULT_SUCCESS : ACT_RESULT_FAILURE;
    if (r == ACT_RESULT_SUCCESS) g_fault_status &= ~FAULT_BIT_SUCTION_CUP;
    else                          g_fault_status |= FAULT_BIT_SUCTION_CUP;
    action_set_result(0x50, r);
    printf("[CMD_0x0050] Suction cup %s: %s\r\n",
           (op == 1) ? "hold" : "release",
           (r == ACT_RESULT_SUCCESS) ? "OK" : "FAIL");
}

/**
 * @brief 0x0010 — 开始流程
 * 按顺序执行 0x61→0x62→0x63→0x64，每步间隔 1s，任一步失败则终止。
 */
static void cmd_start_workflow(void)
{
    uint32_t new_frequency;

    g_stop_requested = 0;
    modbus_reg_set_system_state(SYS_STATE_RUNNING);
    printf("[WORKFLOW] === Start ===\r\n");

    /* Step 1: 0x61 吸膜动作 */
    printf("[WORKFLOW] Step1/4: Suction action (0x61)\r\n");
    cmd_suction_action(0);
    if (g_stop_requested) {
        printf("[WORKFLOW] Stopped at Step1\r\n");
        cmd_stop_workflow();
        return;
    }
    if ((g_action_status & 0xFF) != ACT_RESULT_SUCCESS) {
        printf("[WORKFLOW] Aborted at Step1\r\n");
        modbus_reg_set_system_state(SYS_STATE_WORKFLOW_FAILED);
        cmd_stop_workflow();
        action_set_result(0x10, ACT_RESULT_FAILURE);
        return;
    }
    if (delay_interruptible(1000) != 0) {
        cmd_stop_workflow();
        return;
    }

    /* Step 2: 0x62 铺膜动作 */
    printf("[WORKFLOW] Step2/4: Lay film action (0x62)\r\n");
    cmd_lay_film_action(0);
    if (g_stop_requested) {
        printf("[WORKFLOW] Stopped at Step2\r\n");
        cmd_stop_workflow();
        return;
    }
    if ((g_action_status & 0xFF) != ACT_RESULT_SUCCESS) {
        printf("[WORKFLOW] Aborted at Step2\r\n");
        modbus_reg_set_system_state(SYS_STATE_WORKFLOW_FAILED);
        cmd_stop_workflow();
        action_set_result(0x10, ACT_RESULT_FAILURE);
        return;
    }
    if (delay_interruptible(1000) != 0) {
        cmd_stop_workflow();
        return;
    }

    /* Step 3: 0x63 封膜动作 */
    printf("[WORKFLOW] Step3/4: Seal action (0x63)\r\n");
    cmd_seal_action(0);
    if (g_stop_requested) {
        printf("[WORKFLOW] Stopped at Step3\r\n");
        cmd_stop_workflow();
        return;
    }
    if ((g_action_status & 0xFF) != ACT_RESULT_SUCCESS) {
        printf("[WORKFLOW] Aborted at Step3\r\n");
        modbus_reg_set_system_state(SYS_STATE_WORKFLOW_FAILED);
        cmd_stop_workflow();
        action_set_result(0x10, ACT_RESULT_FAILURE);
        return;
    }
    if (delay_interruptible(1000) != 0) {
        cmd_stop_workflow();
        return;
    }

    /* Step 4: 0x64 取/放孔板 */
    printf("[WORKFLOW] Step4/4: Well plate action (0x64)\r\n");
    cmd_well_plate_action(0);
    if (g_stop_requested) {
        printf("[WORKFLOW] Stopped at Step4\r\n");
        cmd_stop_workflow();
        return;
    }
    if ((g_action_status & 0xFF) != ACT_RESULT_SUCCESS) {
        printf("[WORKFLOW] Aborted at Step4\r\n");
        modbus_reg_set_system_state(SYS_STATE_WORKFLOW_FAILED);
        cmd_stop_workflow();
        action_set_result(0x10, ACT_RESULT_FAILURE);
        return;
    }

    /* 封膜成功，总次数 +1 并保存到 EEPROM */
    new_frequency = g_eeprom_frequency + 1U;
    printf("[PARAM_SAVE] begin reg=0x0080 name=%s value=%lu\r\n",
           eeprom_param_name(0x0080), (unsigned long)new_frequency);
    if (eeprom_write_u32(EEPROM_FREQUENCY, new_frequency) != HAL_OK) {
        printf("[PARAM_SAVE] FAIL reg=0x0080 name=%s value=%lu\r\n",
               eeprom_param_name(0x0080), (unsigned long)new_frequency);
        printf("[WORKFLOW] Save seal count FAIL\r\n");
        modbus_reg_set_system_state(SYS_STATE_WORKFLOW_FAILED);
        action_set_result(0x10, ACT_RESULT_FAILURE);
        return;
    }
    printf("[PARAM_SAVE] OK reg=0x0080 name=%s value=%lu\r\n",
           eeprom_param_name(0x0080), (unsigned long)new_frequency);
    g_eeprom_frequency = new_frequency;
    shadow[0x0080] = (uint16_t)((new_frequency >> 16) & 0xFFFF);
    shadow[0x0081] = (uint16_t)(new_frequency & 0xFFFF);
    printf("[WORKFLOW] Seal count: %lu\r\n", (unsigned long)g_eeprom_frequency);

    modbus_reg_set_system_state(SYS_STATE_FINISHED);
    action_set_result(0x10, ACT_RESULT_SUCCESS);
    printf("[WORKFLOW] === Complete ===\r\n");
}

/* ===================================================================
 *              EEPROM 缓存初始化
 *
 *  上电时调用一次，从 AT24C02 读取所有参数到全局缓存，
 *  并同步更新寄存器影子缓存，后续读写均直接操作缓存。
 * =================================================================== */
void modbus_reg_eeprom_cache_init(void)
{
    uint32_t v;

    if (eeprom_read_u32(EEPROM_ADDR_ZERO, &v) == HAL_OK) {
        g_eeprom_zero = v;
        shadow[0x0070] = (uint16_t)((v >> 16) & 0xFFFF);  // 高16位在低地址 (Big-Endian 字序)
        shadow[0x0071] = (uint16_t)(v & 0xFFFF);          // 低16位在高地址
    }
    if (eeprom_read_u32(EEPROM_SUCK_SEAL, &v) == HAL_OK) {
        g_eeprom_suck_seal = v;
        shadow[0x0072] = (uint16_t)((v >> 16) & 0xFFFF);
        shadow[0x0073] = (uint16_t)(v & 0xFFFF);
    }
    if (eeprom_read_u32(EEPROM_PAVE, &v) == HAL_OK) {
        g_eeprom_pave = v;
        shadow[0x0074] = (uint16_t)((v >> 16) & 0xFFFF);
        shadow[0x0075] = (uint16_t)(v & 0xFFFF);
    }
    if (eeprom_read_u32(EEPROM_GET_PLACE, &v) == HAL_OK) {
        g_eeprom_get_place = v;
        shadow[0x0076] = (uint16_t)((v >> 16) & 0xFFFF);
        shadow[0x0077] = (uint16_t)(v & 0xFFFF);
    }
    if (eeprom_read_u32(EEPROM_TEMP_CTRL, &v) == HAL_OK) {
        g_eeprom_temp_ctrl = (uint16_t)(v & 0xFFFF);
        if (g_eeprom_temp_ctrl > 2500U) {
            g_eeprom_temp_ctrl = 1800U;
        }
        shadow[0x0078] = g_eeprom_temp_ctrl;
        PID_SetTemperature(g_eeprom_temp_ctrl / 10.0f);
    }
    if (eeprom_read_u32(EEPROM_PRESS_TIME, &v) == HAL_OK) {
        g_eeprom_press_time = (uint16_t)(v & 0xFFFF);
        if (g_eeprom_press_time > 60U) {
            g_eeprom_press_time = 0U;
        }
        shadow[0x0079] = g_eeprom_press_time;
    }
    if (eeprom_read_u32(EEPROM_FREQUENCY, &v) == HAL_OK) {
        g_eeprom_frequency = v;
        shadow[0x0080] = (uint16_t)((v >> 16) & 0xFFFF);
        shadow[0x0081] = (uint16_t)(v & 0xFFFF);
    }

    printf("[EEPROM] Cache init: zero=%lu suck_seal=%lu pave=%lu get_place=%lu temp=%u press=%us freq=%lu\r\n",
           (unsigned long)g_eeprom_zero, (unsigned long)g_eeprom_suck_seal,
           (unsigned long)g_eeprom_pave, (unsigned long)g_eeprom_get_place,
           g_eeprom_temp_ctrl, g_eeprom_press_time,
           (unsigned long)g_eeprom_frequency);
}


/* ===================================================================
 *          复合动作异步执行队列
 *
 *  所有耗时调试/流程动作通过消息队列异步执行，
 *  Modbus 写指令立即返回响应，执行结果通过 0x00A0 查询。
 *  涵盖: 0x10/0x12/0x30/0x40/0x41/0x50/0x60~0x64
 * =================================================================== */

osMessageQueueId_t compound_action_queue;

/**
 * @brief 将动作投递到异步队列，立即标记执行中
 * @return WRITE_OK 成功, WRITE_ERR_QUEUE_FULL 队列满
 */
static int action_enqueue(uint8_t addr, uint16_t val16, int32_t val32)
{
    CompoundActionMsg_t msg;
    msg.reg_addr = addr;
    msg.value    = val16;
    msg.value32  = val32;
    if (compound_action_queue == NULL ||
        osMessageQueuePut(compound_action_queue, &msg, 0, 0) != osOK) {
        return WRITE_ERR_QUEUE_FULL;
    }
    action_set_result(addr, ACT_RESULT_EXECUTING);
    return WRITE_OK;
}

void compound_action_task(void *argument)
{
    (void)argument;
    CompoundActionMsg_t msg;

    for (;;) {
        if (osMessageQueueGet(compound_action_queue, &msg, NULL, osWaitForever) == osOK) {
            switch (msg.reg_addr) {
                case 0x10: cmd_start_workflow();                  break;
                case 0x11: cmd_init_workflow();                  break;
                case 0x30: cmd_fujun_move_to(msg.value32);        break;
                case 0x40: cmd_suck_cyl_move(msg.value);        break;
                case 0x41: cmd_seal_cyl_move(msg.value);        break;
                case 0x50: cmd_suction_cup_ctrl(msg.value);     break;
                case 0x12: cmd_stop_workflow();                 break;
                case 0x60: cmd_fujun_home(msg.value);           break;
                case 0x61: cmd_suction_action(msg.value);       break;
                case 0x62: cmd_lay_film_action(msg.value);      break;
                case 0x63: cmd_seal_action(msg.value);          break;
                case 0x64: cmd_well_plate_action(msg.value);    break;
                default: break;
            }
        }
    }
}

/* ===================================================================
 *              公开接口
 * =================================================================== */

const ModbusRegDef_t *modbus_reg_lookup(uint16_t addr)
{
    int left = 0, right = REG_COUNT - 1;
    while (left <= right) {
        int mid = (left + right) / 2;
        uint16_t mid_addr = reg_map[mid].addr;
        if (mid_addr == addr) return &reg_map[mid];
        else if (mid_addr < addr) left = mid + 1;
        else right = mid - 1;
    }
    return NULL;
}

int modbus_reg_check_access(uint16_t addr, uint8_t need_access)
{
    const ModbusRegDef_t *def = modbus_reg_lookup(addr);
    if (def) return (def->access & need_access) ? 1 : 0;

    /* uint32 高半地址：如 0x0071 属于 0x0070 */
    if (addr > 0) {
        def = modbus_reg_lookup(addr - 1);
        if (def && def->type == REG_TYPE_U32 && (def->access & need_access))
            return 1;
    }
    return 0;
}

int modbus_reg_read_value(uint16_t addr, uint16_t *out_value)
{
    const ModbusRegDef_t *def = modbus_reg_lookup(addr);

    /* uint32 幽灵地址：返回缓存的高半部分 */
    if (!def && addr > 0) {
        def = modbus_reg_lookup(addr - 1);
        if (def && def->type == REG_TYPE_U32 && (def->access & 1)) {
            *out_value = shadow[addr];
            return 0;
        }
    }

    if (!def || !(def->access & 1)) return -1;

    switch (addr) {
        case 0x0000: *out_value = FIRMWARE_VERSION_CODE; break;     // 版本信息
        case 0x0001: *out_value = g_slave_addr; break;               // 设备地址
        case 0x0002: *out_value = g_fault_status; break;             // 故障位图
        case 0x0013: *out_value = g_system_state; break;             // 获取系统状态

        case 0x0020: {                                               // 实时温度
            uint16_t t = 0;
            temp_ctrl_get(&t);
            *out_value = t;
            break;
        }
        case 0x0021:
            *out_value = temp_ctrl_is_running() ? 1U : 0U;
            break;
        case 0x00A0: *out_value = g_action_status; break;           // 动作状态

        /* EEPROM 参数 — 从缓存读取（上电时已初始化） */
        case 0x0070:
        case 0x0072:
        case 0x0074:
        case 0x0076:
        case 0x0078:
        case 0x0079:
        case 0x0080:   // 总封膜次数 (uint32, 低16位)
            *out_value = shadow[addr];
            break;

        default: *out_value = shadow[addr]; break;
    }
    return 0;
}

int modbus_reg_write_execute(uint16_t addr, uint16_t value)
{
    const ModbusRegDef_t *def = modbus_reg_lookup(addr);
    if (!def || !(def->access & 2)) return WRITE_ERR_ILLEGAL;

    switch (addr) {
        case 0x0001:  // 修改设备地址
            if (value < 1U || value > 247U) {
                return WRITE_ERR_ILLEGAL;
            }
            g_pending_slave_addr = (uint8_t)value;
            g_slave_addr_pending = 1;
            break;
        /* 0x0010: 开始流程 */
        case 0x0010: {
            if (value != 1U) return WRITE_ERR_ILLEGAL;
            if (g_system_state != SYS_STATE_INITIALIZED &&
                g_system_state != SYS_STATE_FINISHED &&
                g_system_state != SYS_STATE_WORKFLOW_FAILED) {
                printf("[REG_0x0010] System not ready, state=0x%04X\r\n", g_system_state);
                return WRITE_ERR_ILLEGAL;
            }
            int ret = action_enqueue(0x10, 0, 0);
            if (ret != WRITE_OK) { printf("[REG_0x0010] Queue full\r\n"); return ret; }
            printf("[REG_0x0010] Workflow enqueued\r\n");
            break;
        }
        /* 0x0011: 初始化流程（异步入队） */
        case 0x0011: {
            if (value != 1U) return WRITE_ERR_ILLEGAL;
            if (g_system_state == SYS_STATE_RUNNING ||
                g_system_state == SYS_STATE_INITIALIZING) {
                printf("[REG_0x0011] System busy, state=0x%04X\r\n", g_system_state);
                return WRITE_ERR_ILLEGAL;
            }
            int ret = action_enqueue(0x11, 0, 0);
            if (ret != WRITE_OK) { printf("[REG_0x0011] Queue full\r\n"); return ret; }
            printf("[REG_0x0011] Init workflow enqueued\r\n");
            break;
        }
        /* 0x0012: 结束流程（停止） */
        case 0x0012: {
            if (value != 1U) return WRITE_ERR_ILLEGAL;
            if (g_system_state == SYS_STATE_RUNNING ||
                g_system_state == SYS_STATE_INITIALIZING) {
                g_stop_requested = 1;
                action_set_result(0x12, ACT_RESULT_EXECUTING);
                printf("[REG_0x0012] Immediate stop requested\r\n");
                break;
            }
            int ret = action_enqueue(0x12, 0, 0);
            if (ret != WRITE_OK) { printf("[REG_0x0012] Queue full\r\n"); return ret; }
            printf("[REG_0x0012] Stop workflow enqueued\r\n");
            break;
        }

        /* 0x0021: 温控开关 */
        case 0x0021: {
            if (value == 1U) {
                temp_ctrl_start();
            } else if (value == 0U) {
                temp_ctrl_stop();
            } else {
                return WRITE_ERR_ILLEGAL;
            }
            shadow[addr] = value;
            break;
        }

        /* ---- 吸膜电缸 (异步入队) ---- */
        case 0x0040: {
            if (value > 5000) {
                action_set_result(0x40, ACT_RESULT_FAILURE); fault_update(0x40, ACT_RESULT_FAILURE);
                return WRITE_ERR_ILLEGAL;
            }
            int ret = action_enqueue(0x40, value, 0);
            if (ret != WRITE_OK) { action_set_result(0x40, ACT_RESULT_FAILURE); return ret; }
            break;
        }

        /* ---- 封膜电缸 (异步入队) ---- */
        case 0x0041: {
            if (value > 5000) {
                action_set_result(0x41, ACT_RESULT_FAILURE); fault_update(0x41, ACT_RESULT_FAILURE);
                return WRITE_ERR_ILLEGAL;
            }
            int ret = action_enqueue(0x41, value, 0);
            if (ret != WRITE_OK) { action_set_result(0x41, ACT_RESULT_FAILURE); return ret; }
            break;
        }

        /* ---- 电动吸盘 (异步入队) ---- */
        case 0x0050: {
            if (value != 0 && value != 1) {
                action_set_result(0x50, ACT_RESULT_FAILURE); fault_update(0x50, ACT_RESULT_FAILURE);
                return WRITE_ERR_ILLEGAL;
            }
            int ret = action_enqueue(0x50, value, 0);
            if (ret != WRITE_OK) { action_set_result(0x50, ACT_RESULT_FAILURE); return ret; }
            break;
        }

        /* ---- 复合动作（投递到后台异步队列，立即返回） ---- */
        case 0x0060:
        case 0x0061:
        case 0x0062:
        case 0x0063:
        case 0x0064: {
            if (value != 1U) {
                return WRITE_ERR_ILLEGAL;
            }
            int ret = action_enqueue((uint8_t)addr, value, 0);
            if (ret != WRITE_OK) {
                printf("[COMPOUND] Queue full, cmd 0x%02X dropped\r\n", addr);
                action_set_result((uint8_t)addr, ACT_RESULT_FAILURE);
                return ret;
            }
            break;
        }

        /* ---- EEPROM uint16: 写 EEPROM + 同步缓存 ---- */
        case 0x0078:
            if (value > 2500U) return WRITE_ERR_ILLEGAL;
            printf("[PARAM_SAVE] begin reg=0x%04X name=%s value=%u\r\n",
                   addr, eeprom_param_name(addr), value);
            if (eeprom_write_u32(EEPROM_TEMP_CTRL, (uint32_t)value) != HAL_OK) {
                printf("[PARAM_SAVE] FAIL reg=0x%04X name=%s value=%u\r\n",
                       addr, eeprom_param_name(addr), value);
                return WRITE_ERR_DEVICE;
            }
            shadow[addr] = value;
            g_eeprom_temp_ctrl = value;
            PID_SetTemperature(g_eeprom_temp_ctrl / 10.0f);
            printf("[PARAM_SAVE] OK reg=0x%04X name=%s value=%u setpoint=%.1fC\r\n",
                   addr, eeprom_param_name(addr), value, g_eeprom_temp_ctrl / 10.0f);
            break;
        case 0x0079:
            if (value > 60U) return WRITE_ERR_ILLEGAL;
            printf("[PARAM_SAVE] begin reg=0x%04X name=%s value=%u\r\n",
                   addr, eeprom_param_name(addr), value);
            if (eeprom_write_u32(EEPROM_PRESS_TIME, (uint32_t)value) != HAL_OK) {
                printf("[PARAM_SAVE] FAIL reg=0x%04X name=%s value=%u\r\n",
                       addr, eeprom_param_name(addr), value);
                return WRITE_ERR_DEVICE;
            }
            shadow[addr] = value;
            g_eeprom_press_time = value;
            printf("[PARAM_SAVE] OK reg=0x%04X name=%s value=%us\r\n",
                   addr, eeprom_param_name(addr), value);
            break;

        default: return WRITE_ERR_ILLEGAL;
    }
    return WRITE_OK;
}

int modbus_reg_write32_execute(uint16_t start_addr, uint32_t value)
{
    switch (start_addr) {
        case 0x0030: {
            if ((int32_t)value < 0 || (int32_t)value > 110000) {
                action_set_result(0x30, ACT_RESULT_FAILURE);
                fault_update(0x30, ACT_RESULT_FAILURE);
                printf("[CMD_0x0030] Value %ld out of range (0~110000)\r\n", (long)(int32_t)value);
                return WRITE_ERR_ILLEGAL;
            }
            int ret = action_enqueue(0x30, 0, (int32_t)value);
            if (ret != WRITE_OK) {
                action_set_result(0x30, ACT_RESULT_FAILURE);
                fault_update(0x30, ACT_RESULT_FAILURE);
                printf("[CMD_0x0030] Queue full, cmd dropped\r\n");
                return ret;
            }
            printf("[CMD_0x0030] Fujun move to %d enqueued\r\n", (int)value);
            break;
        }
        case 0x0070:
            printf("[PARAM_SAVE] begin reg=0x%04X name=%s value=%lu\r\n",
                   start_addr, eeprom_param_name(start_addr), (unsigned long)value);
            if (eeprom_write_u32(EEPROM_ADDR_ZERO, value) != HAL_OK) {
                printf("[PARAM_SAVE] FAIL reg=0x%04X name=%s value=%lu\r\n",
                       start_addr, eeprom_param_name(start_addr), (unsigned long)value);
                return WRITE_ERR_DEVICE;
            }
            g_eeprom_zero = value;
            shadow[0x0070] = (uint16_t)((value >> 16) & 0xFFFF);
            shadow[0x0071] = (uint16_t)(value & 0xFFFF);
            printf("[PARAM_SAVE] OK reg=0x%04X name=%s value=%lu\r\n",
                   start_addr, eeprom_param_name(start_addr), (unsigned long)value);
            break;
        case 0x0072:
            printf("[PARAM_SAVE] begin reg=0x%04X name=%s value=%lu\r\n",
                   start_addr, eeprom_param_name(start_addr), (unsigned long)value);
            if (eeprom_write_u32(EEPROM_SUCK_SEAL, value) != HAL_OK) {
                printf("[PARAM_SAVE] FAIL reg=0x%04X name=%s value=%lu\r\n",
                       start_addr, eeprom_param_name(start_addr), (unsigned long)value);
                return WRITE_ERR_DEVICE;
            }
            g_eeprom_suck_seal = value;
            shadow[0x0072] = (uint16_t)((value >> 16) & 0xFFFF);
            shadow[0x0073] = (uint16_t)(value & 0xFFFF);
            printf("[PARAM_SAVE] OK reg=0x%04X name=%s value=%lu\r\n",
                   start_addr, eeprom_param_name(start_addr), (unsigned long)value);
            break;
        case 0x0074:
            printf("[PARAM_SAVE] begin reg=0x%04X name=%s value=%lu\r\n",
                   start_addr, eeprom_param_name(start_addr), (unsigned long)value);
            if (eeprom_write_u32(EEPROM_PAVE, value) != HAL_OK) {
                printf("[PARAM_SAVE] FAIL reg=0x%04X name=%s value=%lu\r\n",
                       start_addr, eeprom_param_name(start_addr), (unsigned long)value);
                return WRITE_ERR_DEVICE;
            }
            g_eeprom_pave = value;
            shadow[0x0074] = (uint16_t)((value >> 16) & 0xFFFF);
            shadow[0x0075] = (uint16_t)(value & 0xFFFF);
            printf("[PARAM_SAVE] OK reg=0x%04X name=%s value=%lu\r\n",
                   start_addr, eeprom_param_name(start_addr), (unsigned long)value);
            break;
        case 0x0076:
            printf("[PARAM_SAVE] begin reg=0x%04X name=%s value=%lu\r\n",
                   start_addr, eeprom_param_name(start_addr), (unsigned long)value);
            if (eeprom_write_u32(EEPROM_GET_PLACE, value) != HAL_OK) {
                printf("[PARAM_SAVE] FAIL reg=0x%04X name=%s value=%lu\r\n",
                       start_addr, eeprom_param_name(start_addr), (unsigned long)value);
                return WRITE_ERR_DEVICE;
            }
            g_eeprom_get_place = value;
            shadow[0x0076] = (uint16_t)((value >> 16) & 0xFFFF);
            shadow[0x0077] = (uint16_t)(value & 0xFFFF);
            printf("[PARAM_SAVE] OK reg=0x%04X name=%s value=%lu\r\n",
                   start_addr, eeprom_param_name(start_addr), (unsigned long)value);
            break;
        default: return -1;
    }
    return 0;
}

uint8_t modbus_get_slave_addr(void)
{
    return g_slave_addr;
}

void modbus_apply_pending_slave_addr(void)
{
    if (g_slave_addr_pending) {
        g_slave_addr = g_pending_slave_addr;
        shadow[0x0001] = g_slave_addr;
        g_slave_addr_pending = 0;
        printf("[MODBUS] Slave address changed to %u\r\n", g_slave_addr);
    }
}
