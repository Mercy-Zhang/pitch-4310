/**
 * @file dm4310_driver.h
 * @brief DM4310 电机驱动层(Layer 1)
 *
 * 分层结构:
 *   Layer 0 BSP   — bsp_can (FDCAN 收发), 由驱动层调用, 本模块不重写
 *   Layer 1 驱动  — 本文件: DM4310 协议(反馈解析 / POSVEL 命令打包 / 寄存器读写 / 模式切换)
 *   Layer 2 控制  — dm4310_pitch.c: pitch 轴状态机 + 软件限幅
 *
 * 本层职责边界: 只负责"把物理量搬上 CAN 总线 / 从 CAN 总线解出物理量",
 * 不做任何业务逻辑(摇杆映射、限位、状态机都在上层)。
 */
#ifndef INF_DM4310_DRIVER_H
#define INF_DM4310_DRIVER_H

#include <stdint.h>
#include "bsp_can.h"
#include "daemon.h"

/* 电机默认量程(与达妙寄存器 PMAX/VMAX/TMAX 一致, 上电先用默认值解析首帧) */
#define DM4310_P_DEFAULT   12.5f   /* 位置量程 (rad) */
#define DM4310_V_DEFAULT   200.0f  /* 速度量程 (rad/s) */
#define DM4310_T_DEFAULT   10.0f   /* 扭矩量程 (N·m) */

/* 原生控制模式(寄存器 0x0A, 达妙手册) */
typedef enum {
    DM4310_MODE_MIT    = 1,  /* MIT 模式 */
    DM4310_MODE_POSVEL = 2,  /* 位置速度模式(本工程 pitch 使用) */
    DM4310_MODE_VEL    = 3,  /* 速度模式 */
    DM4310_MODE_POSCUR = 4,  /* 力位混控 */
} DM4310_NativeMode_e;

/* 反馈测量值(单位已换算为 SI) */
typedef struct {
    float position;       /* 位置 (rad), 绝对值编码器 */
    float last_position;  /* 上一拍位置 */
    float velocity;       /* 速度 (rad/s) */
    float torque;         /* 扭矩 (N·m) */
    float t_mos;          /* MOS 温度 (°C) */
    float t_rotor;        /* 转子温度 (°C) */
} DM4310_Measure_s;

/* 软件限幅参数(物理量级, 由控制层配置) */
typedef struct {
    float pos_min;   /* 位置下限 (rad) */
    float pos_max;   /* 位置上限 (rad) */
    float vel_max;   /* 速度上限 (rad/s, POSVEL 梯形速度上限) */
    float torq_max;  /* 扭矩上限 (N·m), 命令打包前钳位 */
} DM4310_Limit_s;

typedef struct {
    DM4310_Measure_s measure;
    DM4310_Limit_s   limit;

    CANInstance    *can;       /* CAN 实例(FDCAN, 反馈经其回调进入) */
    DaemonInstance *daemon;    /* 离线看门狗 */

    float    pid_ref;          /* 目标位置 (rad), 由上层 DMMotorSetRef 写入 */
    uint8_t  native_mode;      /* 目标原生控制模式 */
    uint8_t  enable;           /* 1=使能, 0=停止 */

    /* 寄存器读写响应捕获(异步) */
    uint8_t reg_pending;       /* 1=等待响应中 */
    uint8_t reg_addr;          /* 请求的寄存器地址 */
    union { float f; uint32_t u32; } reg_value;

    /* 诊断计数(Ozone Watch 查看) */
    uint32_t rx_cnt;
    uint32_t tx_ok;
    uint32_t tx_fail;
} DM4310_Instance;

/* ============================ 初始化与生命周期 ============================ */

/**
 * @brief 注册一个 DM4310 电机实例
 * @param can_handle  FDCAN 句柄(&hfdcan1/2/3)
 * @param id          电机 CAN ID(达妙拨码, 反馈帧 = 0x10 + id)
 * @param rx_callback 反馈解析回调(即本驱动的 DM4310_Decode), 也可传 NULL 后手动绑定
 * @return 实例指针(需上层保存)
 */
DM4310_Instance *DM4310_Init(FDCAN_HandleTypeDef *can_handle, uint16_t id);

/* 使能 / 停止(停止时不发命令帧, 电机看门狗自动失能) */
void DM4310_Enable(DM4310_Instance *motor);
void DM4310_Stop(DM4310_Instance *motor);

/* ============================ 目标与软件限幅 ============================ */

/* 设置目标位置; 内部先做软件限幅(位置 clamp)再写入 */
void DM4310_SetRef(DM4310_Instance *motor, float ref);

/* 配置软件限幅参数(在 Init 后、进入控制循环前调用一次) */
void DM4310_SetLimit(DM4310_Instance *motor, float pos_min, float pos_max,
                     float vel_max, float torq_max);

/* ============================ 运行 ============================ */

/* 单次控制: 把 pid_ref 按 POSVEL 协议打包并发送, 建议由 RTOS 任务 500Hz 周期调用 */
void DM4310_Control(DM4310_Instance *motor);

/* 反馈解析: 由 CAN 回调调用, 把 8 字节反馈帧解为物理量 */
void DM4310_Decode(CANInstance *can);

/* 切换原生控制模式(读→失能→写 0x0A→存 flash→确认→使能), 阻塞式 */
void DM4310_SwitchMode(DM4310_Instance *motor, uint8_t native_mode);

/* 使能/停止原始命令(协议层) */
void DM4310_SendCommand(DM4310_Instance *motor, uint8_t cmd);

#endif /* INF_DM4310_DRIVER_H */
