/**
 * @file robot_def.h
 * @brief pitch 单轴参数与共享数据结构
 *
 * 机械/接线参数集中在这里; 状态枚举与遥控结构体是各 FSM 任务的公共契约。
 */
#ifndef INF_ROBOT_DEF_H
#define INF_ROBOT_DEF_H

#include <stdint.h>

/* ============================== 云台 pitch ============================== */

/* pitch 电机 CAN 地址 */
#define CFG_PITCH_DM_ID    2   /* DM4310, FDCAN1, POSVEL 模式, tx=0x102 rx=0x12 */

/* pitch 水平位置: 云台水平时电机绝对位置(rad), 机械改动需重新标定 */
#define CFG_PITCH_HORIZON_RAD   0.435f
/*pitch轴标定*/
/* pitch 机械行程(rad) */
#define CFG_PITCH_MAX_RAD       0.75f
#define CFG_PITCH_MIN_RAD       0.14f

/* ============================== 拨盘 M2006 ============================== */

/* 拨盘 M2006 CAN ID (FDCAN1) */
#define CFG_LOADER_M2006_ID  6   /* M2006, DJI 协议: rx=0x206 (拨码 6), 走 0x1FF 命令组 */

/* ============================== 状态枚举 ============================== */

typedef enum {
    GIMBAL_STATE_DISABLE = 0,  /* 急停, pitch 保持最后位置 */
    GIMBAL_STATE_FOLLOW,       /* 手动跟随: pitch 摇杆 → 位置 */
} gimbal_state_e;

/* ============================== 遥控器 ============================== */
// #define SBUS
#define DBUS
typedef struct {
    int16_t gimbal_yaw;   /* 云台 yaw 摇杆(右手水平), -1000~1000 (未用) */
    int16_t gimbal_pitch; /* 云台 pitch 摇杆(右手竖直), -1000~1000 */
    int16_t chassis_x;    /* (未用) */
    int16_t chassis_y;    /* (未用) */
    uint8_t switch_right; /* DBUS/DT7: 右开关 S2 原始值(1上/3中/2下) */
    uint8_t switch_left;  /* DBUS/DT7: 左开关 S1 原始值(1上/3中/2下), pitch 急停 */
    uint8_t sw_estop;    /* SBUS/天地飞专用: 急停(两段): 0下=急停, 2上=使能 */
    uint8_t sw_mode;     /* SBUS/天地飞专用: 模式(三段) */
    uint8_t sw_friction; /* SBUS/天地飞专用 */
    uint8_t sw_loader;   /* SBUS/天地飞专用 */
    int16_t wheel;       /* DBUS/DT7: 拨轮; SBUS: 旋钮(未用) */
    uint8_t online;      /* 1=在线 */
} Remote_t;

/* ============================== 共享输入 ============================== */

/* 由 remote 模块周期性刷新 */
typedef struct {
    Remote_t remote;
} Robot_Input_t;

extern Robot_Input_t g_input;

#endif /* INF_ROBOT_DEF_H */
