/**
 * @file dm4310_pitch.c
 * @brief DM4310 pitch 轴控制层实现(Layer 2) — 摇杆控制
 *
 * 摇杆控制: 遥控器 pitch 摇杆(-1000~1000)线性映射到机械行程内绝对位置。
 * POSVEL 模式下电机自带位置速度双环, 松杆后停在当前角度(位置保持)。
 *
 * 软件限幅集中点:
 *   位置  dm_clampf(target, MIN_RAD, MAX_RAD)
 *   速度  梯形速度上限 = CFG_PITCH_VEL_MAX(恒正)
 *   摇杆  ratio = clamp((stick+1000)/2000, 0, 1)
 *   启动/离线 保持当前测量位置, 防止 POSVEL 上电/重连瞬间乱跳
 */
#include "dm4310_pitch.h"
#include "dm4310_driver.h"
#include "daemon.h"

static DM4310_Instance *pitch;
static uint8_t          enabled = 0;
static uint16_t         startup_cnt = 0;

/* 参考工程遥控器摇杆输入(接入遥控模块后替换为 g_input.remote.gimbal_pitch) */
extern volatile int16_t remote_pitch_stick;   /* -1000~1000 */

/* ============================ 软件限幅工具 ============================ */

static inline float pitch_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* 摇杆 → 比例(软件限幅 #3: clamp 到 [0,1]) */
static float pitch_stick_ratio(int16_t stick)
{
    float ratio = ((float)stick + 1000.0f) / 2000.0f;
    return pitch_clampf(ratio, 0.0f, 1.0f);
}

/* 摇杆 → 目标位置(软件限幅 #1: 最终目标 clamp 到机械行程) */
static float pitch_calc_target(void)
{
    float ratio = pitch_stick_ratio(remote_pitch_stick);
    float target = CFG_PITCH_MIN_RAD + ratio * (CFG_PITCH_MAX_RAD - CFG_PITCH_MIN_RAD);
    return pitch_clampf(target, CFG_PITCH_MIN_RAD, CFG_PITCH_MAX_RAD);
}

/* ============================ 生命周期 ============================ */

void DM4310Pitch_Init(void)
{
    pitch = DM4310_Init(&hcan2, CFG_PITCH_DM_ID);

    /* 软件限幅配置(集中配置, 只此一处) */
    DM4310_SetLimit(pitch,
                    CFG_PITCH_MIN_RAD, CFG_PITCH_MAX_RAD,  /* 位置限幅 */
                    CFG_PITCH_VEL_MAX,                     /* 速度限幅 */
                    CFG_PITCH_TORQ_MAX);                   /* 扭矩限幅 */

    DM4310Pitch_Enable();
}

void DM4310Pitch_Enable(void)
{
    enabled = 1;
    startup_cnt = 0;   /* 重新使能后重新进入启动保持 */
}

void DM4310Pitch_Disable(void)
{
    enabled = 0;
    startup_cnt = 0;
}

DM4310_Instance *DM4310Pitch_GetInstance(void)
{
    return pitch;
}

/* ============================ 周期任务 ============================ */

void DM4310Pitch_Task(void)
{
    if (pitch == NULL)
        return;

    /* 急停: 失能, 不发命令(电机看门狗超时自停) */
    if (!enabled)
    {
        DM4310_Stop(pitch);
        return;
    }
    DM4310_Enable(pitch);

    /* 启动/掉线保持(#4): 上电前 N 拍或离线期间, 目标=当前测量位置, 防止乱跳 */
    if (!DaemonIsOnline(pitch->daemon) || startup_cnt < CFG_PITCH_STARTUP_HOLD)
    {
        if (DaemonIsOnline(pitch->daemon))
            startup_cnt++;
        DM4310_SetRef(pitch, pitch->measure.position);   /* 保持当前位置 */
        return;
    }

    /* 正常摇杆控制: 生成目标(内部已做位置软件限幅) → 下发 */
    DM4310_SetRef(pitch, pitch_calc_target());
}

/* 驱动层发送任务(高频, 独立于状态机) */
void DM4310Pitch_Control(void)
{
    if (pitch != NULL)
        DM4310_Control(pitch);
}
