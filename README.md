# 4310 pitch 单轴电控（遥控器控制抬头/低头）

> STM32H723 + 达妙 DM4310(POSVEL 原生位置速度模式) + FreeRTOS，分层结构与
> `D:\STM32\Infantry-new` 一致，只保留 pitch 单轴 + 遥控器。

## 分层结构

```
application/        业务逻辑(FSM): gimbal(pitch 控制)、loader(拨盘 M2006)、robot(初始化/主循环)
├── gimbal.c        遥控器摇杆 → pitch 目标位置 + 上电/掉线保持 + 急停
├── loader.c        拨盘 M2006 控制(拨轮连发 + 卡弹反转保护)
├── robot.c         RobotInit / RobotTask(remote → gimbal / loader)
├── robot_def.h     pitch 机械参数 + M2006 地址 + Remote_t + g_input 契约
└── robot_task.h    FreeRTOS 任务创建(robot 100Hz / motor 1kHz / daemon 100Hz / DM 独立任务)
modules/            驱动与算法(复用, 不写业务)
├── remote/         remote_control: DBUS/SBUS 接收机解析
├── master_machine/ remote: rc_data → g_input.remote 适配层
├── motor/DMmotor/  DM4310 驱动: 反馈解析 / POSVEL 命令打包 / 模式切换
├── motor/DJImotor/ dji_motor: DJI 电机驱动(M2006 速度环) + motor_task(1kHz 闭环)
├── daemon/         离线看门狗
├── algorithm/      controller(PID) + user_lib(数学工具)
└── general_def.h / memory.h / motor_def.h
bsp/                硬件抽象: can(FDCAN)、usart(DMA+IDLE)、dwt、log(RTT)
Core/ Drivers/ Middlewares/ USB_DEVICE/    CubeMX 生成, 原样保留
```

## 遥控器 → pitch 数据流

```
接收机(DBUS/DT7) ──UART5──> remote_control.c(解析 18 字节帧)
   └─> remote.c: gimbal_pitch = rocker_r1(右手竖直, -660~660) * 1000/660  → -1000~1000
        └─> g_input.remote.gimbal_pitch
             └─> gimbal.c: ratio=(stick+1000)/2000 → pos_ref = MIN + ratio*(MAX-MIN)
                  └─> DMMotorSetRef() → DM4310 POSVEL(位置速度双环, 松杆保持)
```

- 摇杆最上 → `CFG_PITCH_MAX_RAD`(抬头)，最下 → `CFG_PITCH_MIN_RAD`(低头)，中位 → 行程中点。
- POSVEL 电机自带位置速度双环，**松杆后停在当前角度**。
- 急停：DBUS 左开关 S1 拨到上(`switch_left==1`)→ pitch 保持不动。

## 遥控器协议

- 默认 `DBUS`(DJI DT7/DR16，18 字节帧)。SBUS(天地飞)代码完整保留，
  切换时把 `modules/remote/remote_control.h` 与 `application/robot_def.h` 里的
  `#define DBUS` 改成 `#define SBUS` 即可。

## 拨盘 M2006(移植自 Infantry-new 的 shoot.c)

- M2006 + C610：DJI 速度环(`SPEED_LOOP | CURRENT_LOOP`)，FDCAN1，拨码 `CFG_LOADER_M2006_ID`，
  反馈帧 `0x200+id`，走 `0x1FF` 命令组。
- 控制源(DBUS/DT7)：拨轮 `wheel > 450` → 连发(拨盘起转)，`wheel < -450` 或左开关急停 → 停。
- 卡弹保护：拨盘电流超 `JAM_CURRENT_THRESHOLD` → 反转退弹 `REVERSE_TIME`，时间到自动恢复。
- 标定项集中在 `application/loader.c`：`loader_fire_speed`(转速)、`JAM_CURRENT_THRESHOLD`、
  `REVERSE_SPEED`、`REVERSE_TIME`，以及 `motor_reverse_flag`(转向，上机实测)。

## 上机标定

`application/robot_def.h` 中 pitch 机械参数：

| 宏 | 含义 |
|----|------|
| `CFG_PITCH_DM_ID` | DM4310 电机 CAN ID(达妙拨码, 反馈帧=0x10+id) |
| `CFG_PITCH_HORIZON_RAD` | 云台水平时电机绝对位置(rad)，上机读 `measure.position` 填写 |
| `CFG_PITCH_MIN_RAD` / `CFG_PITCH_MAX_RAD` | 机械行程下限/上限(rad)，软件限幅 |

## 编译

工程根目录(本目录)执行 `mingw32-make`，生成 `build/infantry.elf/.hex/.bin`。
烧录：`mingw32-make download_jlink`(需配置 `stm32.jflash` 与 JFlash 路径)。
