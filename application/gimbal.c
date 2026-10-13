/**
 * @file gimbal.c
 * @brief 云台控制(仅 pitch 单轴, 遥控器控制抬头/低头)
 *
 * pitch: DM4310 POSVEL 原生位置速度模式, 直接发位置指令。
 * 遥控器 pitch 摇杆(g_input.remote.gimbal_pitch, -1000~1000)线性映射到
 * 机械行程 [CFG_PITCH_MIN_RAD, CFG_PITCH_MAX_RAD]; 松杆后 POSVEL 电机
 * 自带位置速度双环, 停在当前角度(位置保持)。
 *
 * 模式(急停开关, 两态):
 *   DISABLE 急停: pitch 不更新目标(保持最后位置)
 *   FOLLOW  遥控: pitch 摇杆 → 位置
 */
#include "gimbal.h"
#include "dmmotor.h"
#include "robot_def.h"
#include "daemon.h"

static DMMotorInstance *pitch_motor;   /* DM4310, POSVEL */

static gimbal_state_e state = GIMBAL_STATE_DISABLE;

/* ============================ 初始化 ============================ */

void GimbalInit(void)
{
    /* pitch DM4310 — POSVEL 原生位置速度模式 */
    Motor_Init_Config_s pitch_cfg = {
        .can_init_config = {.can_handle = &hcan1,
                            .tx_id = CFG_PITCH_DM_ID,
                            .rx_id = 0x10 + CFG_PITCH_DM_ID},
    };
    pitch_motor = DMMotorInit(&pitch_cfg);
    pitch_motor->p_max = 3.14159265f;
    pitch_motor->v_max = 30.0f;
    pitch_motor->t_max = 18.0f;
    pitch_motor->native_mode = DM_NATIVE_MODE_POSVEL;
    DMMotorSetRef(pitch_motor, CFG_PITCH_HORIZON_RAD);
}

/* ============================ 状态迁移 ============================ */

/* 急停开关 → 控制模式 */
static gimbal_state_e GimbalDecode(void)
{
#ifdef SBUS
    if (g_input.remote.sw_estop == 2)     return GIMBAL_STATE_DISABLE;   /* 急停 */
#endif
#ifdef DBUS
    if (g_input.remote.switch_left == 1)  return GIMBAL_STATE_DISABLE;   /* 急停(上) */
#endif
    return GIMBAL_STATE_FOLLOW;
}

/* ============================ Pitch 轴处理 ============================ */

/* Pitch 轴: POSVEL 原生位置速度模式, 摇杆映射到角度 */
static void GimbalHandlePitch(void)
{
    if (pitch_motor == NULL)
        return;

    static uint16_t startup_cnt = 0;
    static float    pos_ref = 0.0f;

    /* 上电/掉线: 保持当前位置, 防止 POSVEL 乱跳 */
    if (DaemonIsOnline(pitch_motor->motor_daemon) == 0)
    {
        startup_cnt = 0;
        pos_ref = pitch_motor->measure.position;
        DMMotorSetRef(pitch_motor, pos_ref);
        return;
    }
    startup_cnt++;
    if (startup_cnt < 50)  /* 上电后前 50 拍保持当前位 */
    {
        pos_ref = pitch_motor->measure.position;
        DMMotorSetRef(pitch_motor, pos_ref);
        return;
    }

    /* 摇杆线性映射到机械范围 */
    float ratio = ((float)g_input.remote.gimbal_pitch + 1000.0f) / 2000.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < 0.0f) ratio = 0.0f;
    pos_ref = CFG_PITCH_MIN_RAD + ratio * (CFG_PITCH_MAX_RAD - CFG_PITCH_MIN_RAD);

    /* 机械限位 */
    if (pos_ref > CFG_PITCH_MAX_RAD) pos_ref = CFG_PITCH_MAX_RAD;
    if (pos_ref < CFG_PITCH_MIN_RAD) pos_ref = CFG_PITCH_MIN_RAD;

    DMMotorSetRef(pitch_motor, pos_ref);
}

/* ============================ 任务入口 ============================ */

void GimbalTask(void)
{
    state = GimbalDecode();  /* ① 急停开关 → 状态 */

    /* ② 失能态: pitch 不更新目标(保持最后位置), 立即返回 */
    if (state == GIMBAL_STATE_DISABLE)
        return;

    /* ③ Pitch 轴: 遥控器摇杆控制抬头/低头 */
    GimbalHandlePitch();
}
