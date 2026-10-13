/**
 * @file dm4310_pitch.h
 * @brief DM4310 pitch 轴控制层(Layer 2) — 摇杆控制
 *
 * 分层结构:
 *   Layer 0 BSP   — bsp_can
 *   Layer 1 驱动  — dm4310_driver.{h,c} (协议 + 物理量换算)
 *   Layer 2 控制  — 本文件: 摇杆 → 目标位置 + 软件限幅
 *
 * 控制方式: 遥控器 pitch 摇杆(-1000~1000)线性映射到机械行程内绝对位置,
 * 松杆后 POSVEL 模式电机自带位置保持, 停在松杆时的角度。
 *
 * 软件限幅(集中在本层, 见 dm4310_pitch.c 实现):
 *   1) 位置限幅  — 目标位置 clamp 到机械行程 [CFG_PITCH_MIN_RAD, CFG_PITCH_MAX_RAD]
 *   2) 速度限幅  — POSVEL 梯形速度上限 clamp 到 [0, CFG_PITCH_VEL_MAX]
 *   3) 摇杆限幅  — 摇杆线性映射比例 clamp 到 [0, 1]
 *   4) 上电/掉线 — 上电前 N 拍与离线期间保持当前位置(目标=当前测量值)
 */
#ifndef INF_DM4310_PITCH_H
#define INF_DM4310_PITCH_H

#include <stdint.h>
#include "dm4310_driver.h"

/* ---- pitch 机械参数(上机标定, 与参考工程 robot_def.h 一致) ---- */
#define CFG_PITCH_DM_ID         2        /* DM4310 电机 CAN ID(达妙拨码) */
#define CFG_PITCH_HORIZON_RAD   0.435f   /* 云台水平时电机绝对位置(rad), 上机标定 */
#define CFG_PITCH_MIN_RAD       0.14f    /* 机械下限(rad), 软件限幅下限 */
#define CFG_PITCH_MAX_RAD       0.75f    /* 机械上限(rad), 软件限幅上限 */
#define CFG_PITCH_VEL_MAX       30.0f    /* 梯形速度上限(rad/s), 速度软件限幅 */
#define CFG_PITCH_TORQ_MAX      18.0f    /* 扭矩上限(N·m) */
#define CFG_PITCH_STARTUP_HOLD  50       /* 上电后保持当前位的拍数(100Hz) */

/* 初始化: 创建 DM4310 实例, 配置软件限幅 */
void DM4310Pitch_Init(void);

/* 周期任务(建议 100Hz): 摇杆映射 + 软件限幅 + 下发目标 */
void DM4310Pitch_Task(void);

/* 驱动层控制发送任务(建议 500Hz): 打包并发送 POSVEL 命令帧 */
void DM4310Pitch_Control(void);

/* 使能/急停(由上层急停开关调用) */
void DM4310Pitch_Enable(void);
void DM4310Pitch_Disable(void);

/* 获取实例指针(供上层读反馈 / 调参) */
DM4310_Instance *DM4310Pitch_GetInstance(void);

#endif /* INF_DM4310_PITCH_H */
