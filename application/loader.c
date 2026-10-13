/**
 * @file loader.c
 * @brief 拨盘 M2006 控制(移植自 Infantry-new shoot.c 的拨盘部分)
 *
 * M2006 + C610: DJI 速度环(SPEED_LOOP + CURRENT_LOOP), 拨码 6, 走 0x1FF 命令组, rx=0x206。
 * 控制源(DBUS/DT7): 拨轮 wheel > 450 → 连发(拨盘起转), wheel < -450 或 急停 → 停。
 * 卡弹: 拨盘电流超阈值 → 反转退弹, 时间到恢复。
 */
#include "loader.h"
#include "dji_motor.h"
#include "robot_def.h"
#include "bsp_dwt.h"

static DJIMotorInstance *loader;   /* M2006 拨盘 */

/* 卡弹反转参数(需实测标定) */
#define JAM_CURRENT_THRESHOLD  6000.0f  /* 卡弹电流阈值(M2006 原始 int16) */
#define REVERSE_SPEED    -800.0f       /* 反转速度 °/s(转子, 负=反转) */
#define REVERSE_TIME     500.0f        /* 反转持续时间 ms */

/* 拨盘发弹参数(Ozone 可调, 需按机械实测标定) */
volatile float loader_fire_speed = 12000.0f;  /* 拨盘转速 °/s(转子, M2006) */

/* 拨盘状态机 */
static enum {
    LOADER_NORMAL,
    LOADER_REVERSING,
} loader_state = LOADER_NORMAL;
static float reverse_start_time = 0;

/* ============================ 初始化 ============================ */

void LoaderInit(void)
{
    /* 拨盘 M2006 — DJI 速度环(参考 shoot.c 里的 M2006 配置) */
    Motor_Init_Config_s loader_config = {
        .can_init_config = {.can_handle = &hcan1, .tx_id = CFG_LOADER_M2006_ID},
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10.0f, .Ki = 0.0f, .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000.0f,
                .MaxOut = 9000.0f, .MaxOut_ = -9000.0f,
            },
            .current_PID = {
                .Kp = 0.7f, .Ki = 0.0f, .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000.0f,
                .MaxOut = 9000.0f, .MaxOut_ = -9000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,   /* 上机标定 */
        },
        .motor_type = M2006,
    };
    loader = DJIMotorInit(&loader_config);
    DJIMotorSetRef(loader, 0);
}

/* ============================ 任务入口 ============================ */

void LoaderTask(void)
{
    if (loader == NULL)
        return;

    /* ====== 输入映射(DBUS/DT7): 拨轮向上=连发, 向下=急停 ====== */
    uint8_t estop_active = (g_input.remote.switch_left == 1) || (g_input.remote.wheel < -450);
    uint8_t fire          = (g_input.remote.wheel > 450);

    /* ====== 急停: 拨盘停 ====== */
    if (estop_active)
    {
        DJIMotorSetRef(loader, 0);
        loader_state = LOADER_NORMAL;
        return;
    }

    /* ====== 卡弹回转保护 ====== */
    if (loader_state == LOADER_REVERSING)
    {
        DJIMotorSetRef(loader, REVERSE_SPEED);
        if (DWT_GetTimeline_ms() - reverse_start_time > REVERSE_TIME)
            loader_state = LOADER_NORMAL;
        return;
    }
    if (loader->measure.real_current > JAM_CURRENT_THRESHOLD)
    {
        loader_state = LOADER_REVERSING;
        reverse_start_time = DWT_GetTimeline_ms();
        DJIMotorSetRef(loader, REVERSE_SPEED);
        return;
    }

    /* ====== 连发: 拨盘起转; 否则停 ====== */
    DJIMotorSetRef(loader, fire ? loader_fire_speed : 0);
}
