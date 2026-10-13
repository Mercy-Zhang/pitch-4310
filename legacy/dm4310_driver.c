/**
 * @file dm4310_driver.c
 * @brief DM4310 电机驱动层实现(Layer 1)
 *
 * 达妙 DM4310 POSVEL 原生位置速度模式协议要点(达妙手册):
 *   - 命令帧 CAN ID = 0x100*(mode-1) + 电机ID, 如 POSVEL(mode=2)+ID2 → 0x102
 *   - 命令帧 8 字节直存 float(小端): D[0:3]=p_des(rad), D[4:7]=v_des(rad/s 梯形速度上限)
 *   - 反馈帧 CAN ID = 0x10 + 电机ID(如 ID2 → 0x12), 8 字节位打包:
 *       D[0]=ID, D[1:2]=POS(16bit), D[3:4高4]=VEL(12bit),
 *       D[4低4:5]=TORQUE(12bit), D[6]=T_MOS, D[7]=T_Rotor
 *   - 寄存器读写走 0x7FF 广播帧(D[2]=0x55 写 / 0x33 读 / 0xAA 存 flash)
 *
 * 本层只做协议与物理量换算, 软件限幅在 DM4310_SetRef/Control 中由 limit 结构体钳位。
 */
#include "dm4310_driver.h"
#include "stdlib.h"
#include "string.h"
#include "cmsis_os.h"

/* ============================ 内部工具 ============================ */

/* 浮点限幅 */
static inline float dm_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* uint 与 float 映射(反馈 12/16 位解析用, 量程必须与寄存器一致) */
static uint16_t dm_float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return (uint16_t)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

static float dm_uint_to_float(int x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return (float)x * span / (float)((1 << bits) - 1) + x_min;
}

/* ============================ 协议命令 ============================ */

/* 发送一帧"电机模式命令"(D[0:6]=0xFF, D[7]=命令字), 用于使能/停止/清错误/归零 */
void DM4310_SendCommand(DM4310_Instance *motor, uint8_t cmd)
{
    memset(motor->can->tx_buff, 0xFF, 7);
    motor->can->tx_buff[7] = cmd;
    CANTransmit(motor->can, 1);
}

/* 广播写寄存器(0x7FF), 不破坏原 tx_buff / Identifier */
static void dm_write_reg(DM4310_Instance *motor, uint8_t reg, uint32_t value)
{
    uint8_t tx[8];
    uint16_t id = motor->can->tx_id;

    tx[0] = (uint8_t)(id & 0xFF);
    tx[1] = (uint8_t)((id >> 8) & 0xFF);
    tx[2] = 0x55;                 /* 写命令标志 */
    tx[3] = reg;                  /* 寄存器地址 */
    tx[4] = (uint8_t)(value & 0xFF);
    tx[5] = (uint8_t)((value >> 8) & 0xFF);
    tx[6] = (uint8_t)((value >> 16) & 0xFF);
    tx[7] = (uint8_t)((value >> 24) & 0xFF);

    uint8_t  save_buf[8];
    uint32_t save_id = motor->can->txconf.Identifier;
    memcpy(save_buf, motor->can->tx_buff, 8);
    memcpy(motor->can->tx_buff, tx, 8);

    motor->can->txconf.Identifier = 0x7FF;
    CANTransmit(motor->can, 1);

    memcpy(motor->can->tx_buff, save_buf, 8);
    motor->can->txconf.Identifier = save_id;
}

/* 广播读寄存器(0x7FF), 响应经 DM4310_Decode 的 reg_pending 分支捕获 */
static void dm_read_reg(DM4310_Instance *motor, uint8_t reg)
{
    motor->reg_pending = 1;
    motor->reg_addr    = reg;

    uint8_t tx[8];
    uint16_t id = motor->can->tx_id;
    tx[0] = (uint8_t)(id & 0xFF);
    tx[1] = (uint8_t)((id >> 8) & 0xFF);
    tx[2] = 0x33;                 /* 读命令标志 */
    tx[3] = reg;
    memset(&tx[4], 0, 4);

    uint8_t  save_buf[8];
    uint32_t save_id = motor->can->txconf.Identifier;
    memcpy(save_buf, motor->can->tx_buff, 8);
    memcpy(motor->can->tx_buff, tx, 8);

    motor->can->txconf.Identifier = 0x7FF;
    CANTransmit(motor->can, 1);

    memcpy(motor->can->tx_buff, save_buf, 8);
    motor->can->txconf.Identifier = save_id;
}

/* 存 flash(0x7FF, D[2]=0xAA D[3]=0x01), 使寄存器写入掉电不丢 */
static void dm_save_flash(DM4310_Instance *motor)
{
    uint8_t tx[8];
    uint16_t id = motor->can->tx_id;
    tx[0] = (uint8_t)(id & 0xFF);
    tx[1] = (uint8_t)((id >> 8) & 0xFF);
    tx[2] = 0xAA;
    tx[3] = 0x01;
    memset(&tx[4], 0, 4);

    uint8_t  save_buf[8];
    uint32_t save_id = motor->can->txconf.Identifier;
    memcpy(save_buf, motor->can->tx_buff, 8);
    memcpy(motor->can->tx_buff, tx, 8);

    motor->can->txconf.Identifier = 0x7FF;
    CANTransmit(motor->can, 1);

    memcpy(motor->can->tx_buff, save_buf, 8);
    motor->can->txconf.Identifier = save_id;
}

/* 把 tx 的 Identifier 切到当前原生模式对应的命令 ID(如 POSVEL→0x100+id) */
static void dm_set_mode_txid(DM4310_Instance *motor)
{
    if (motor->native_mode >= DM4310_MODE_POSVEL)
        motor->can->txconf.Identifier = 0x100 * (motor->native_mode - 1) + motor->can->tx_id;
}

/* 阻塞等待寄存器响应, 返回 0=超时 */
static uint8_t dm_wait_reg(DM4310_Instance *motor, uint16_t max_wait_ms)
{
    for (uint16_t w = 0; w < max_wait_ms; w++)
    {
        if (motor->reg_pending == 0) return 1;
        osDelay(1);
    }
    return 0;
}

/* ============================ 反馈解析 ============================ */

void DM4310_Decode(CANInstance *can)
{
    uint8_t *rx = can->rx_buff;
    DM4310_Instance *motor = (DM4310_Instance *)can->id;

    motor->rx_cnt++;

    /* 寄存器响应帧拦截: D[2]=0x33/0x55/0xAA, 与反馈帧位打包格式不同 */
    if (motor->reg_pending && (rx[2] == 0x33 || rx[2] == 0x55 || rx[2] == 0xAA))
    {
        memcpy(&motor->reg_value.u32, &rx[4], 4);
        motor->reg_pending = 0;
        return;   /* 不污染 measure */
    }

    DaemonReload(motor->daemon);

    DM4310_Measure_s *m = &motor->measure;
    uint16_t tmp;

    m->last_position = m->position;
    tmp = (uint16_t)((rx[1] << 8) | rx[2]);
    m->position = dm_uint_to_float(tmp, -DM4310_P_DEFAULT, DM4310_P_DEFAULT, 16);

    tmp = (uint16_t)((rx[3] << 4) | (rx[4] >> 4));
    m->velocity = dm_uint_to_float(tmp, -DM4310_V_DEFAULT, DM4310_V_DEFAULT, 12);

    tmp = (uint16_t)(((rx[4] & 0x0F) << 8) | rx[5]);
    m->torque = dm_uint_to_float(tmp, -DM4310_T_DEFAULT, DM4310_T_DEFAULT, 12);

    m->t_mos   = (float)rx[6];
    m->t_rotor = (float)rx[7];
}

/* ============================ 初始化 ============================ */

DM4310_Instance *DM4310_Init(FDCAN_HandleTypeDef *can_handle, uint16_t id)
{
    DM4310_Instance *motor = (DM4310_Instance *)malloc(sizeof(DM4310_Instance));
    memset(motor, 0, sizeof(DM4310_Instance));

    /* 软件限幅默认值: 上电先用宽范围, 待控制层 DM4310_SetLimit 覆盖 */
    motor->limit.pos_min  = -DM4310_P_DEFAULT;
    motor->limit.pos_max  =  DM4310_P_DEFAULT;
    motor->limit.vel_max  =  30.0f;   /* 梯形速度上限占位, 上层按需改 */
    motor->limit.torq_max =  18.0f;

    motor->native_mode = DM4310_MODE_POSVEL;
    motor->enable      = 1;

    /* CAN 注册: 反馈经 DM4310_Decode 进入 */
    CAN_Init_Config_s cfg = {
        .can_handle          = can_handle,
        .tx_id               = id,          /* 命令 ID 在切换模式时按 0x100*(mode-1)+id 计算 */
        .rx_id               = 0x10 + id,   /* 反馈 ID */
        .can_module_callback = DM4310_Decode,
        .id                  = motor,
    };
    motor->can = CANRegister(&cfg);

    /* 离线看门狗 */
    Daemon_Init_Config_s dcfg = {
        .callback     = NULL,
        .owner_id     = motor,
        .reload_count = 10,
        .name         = "dm4310",
    };
    motor->daemon = DaemonRegister(&dcfg);

    return motor;
}

/* ============================ 启停 ============================ */

void DM4310_Enable(DM4310_Instance *motor)
{
    motor->enable = 1;
}

void DM4310_Stop(DM4310_Instance *motor)
{
    motor->enable = 0;
}

/* ============================ 软件限幅 ============================ */

void DM4310_SetLimit(DM4310_Instance *motor, float pos_min, float pos_max,
                     float vel_max, float torq_max)
{
    motor->limit.pos_min  = pos_min;
    motor->limit.pos_max  = pos_max;
    motor->limit.vel_max  = vel_max;
    motor->limit.torq_max = torq_max;
}

/* 设置目标位置: 先位置软件限幅, 再写入 pid_ref */
void DM4310_SetRef(DM4310_Instance *motor, float ref)
{
    motor->pid_ref = dm_clampf(ref, motor->limit.pos_min, motor->limit.pos_max);
}

/* ============================ 控制发送 ============================ */

void DM4310_Control(DM4310_Instance *motor)
{
    /* 停止: 不发命令帧, 电机看门狗(~20ms)超时后自行失能 */
    if (!motor->enable)
    {
        memset(motor->can->tx_buff, 0, 8);
        return;
    }

    dm_set_mode_txid(motor);

    /* POSVEL: D[0:3]=p_des(位置限幅), D[4:7]=v_des(速度限幅) */
    if (motor->native_mode == DM4310_MODE_POSVEL)
    {
        float p_cmd = dm_clampf(motor->pid_ref, motor->limit.pos_min, motor->limit.pos_max);
        float v_cmd = motor->limit.vel_max;   /* 梯形速度上限, 恒正 */

        memcpy(&motor->can->tx_buff[0], &p_cmd, 4);
        memcpy(&motor->can->tx_buff[4], &v_cmd, 4);
    }
    else
    {
        /* MIT 模式: 位打包 8 字节(扭矩=Kp*e + Kd*de + t_ff), 本工程 pitch 不用, 保留骨架 */
        float p_cmd = dm_clampf(motor->pid_ref, motor->limit.pos_min, motor->limit.pos_max);
        motor->can->tx_buff[0] = (uint8_t)(dm_float_to_uint(p_cmd, -DM4310_P_DEFAULT, DM4310_P_DEFAULT, 16) >> 8);
        motor->can->tx_buff[1] = (uint8_t)(dm_float_to_uint(p_cmd, -DM4310_P_DEFAULT, DM4310_P_DEFAULT, 16));
        /* Kp/Kd/vel/torque 按需扩展 */
        motor->can->tx_buff[2] = 0;
        motor->can->tx_buff[3] = 0;
        motor->can->tx_buff[4] = 0;
        motor->can->tx_buff[5] = 0;
        motor->can->tx_buff[6] = 0;
        motor->can->tx_buff[7] = 0;
    }

    if (CANTransmit(motor->can, 1))
        motor->tx_ok++;
    else
        motor->tx_fail++;
}

/* ============================ 模式切换 ============================ */

void DM4310_SwitchMode(DM4310_Instance *motor, uint8_t native_mode)
{
    uint32_t target = (uint32_t)native_mode;

    /* 1. 读当前模式, 已一致则跳过(省一次 flash 擦写) */
    dm_read_reg(motor, 0x0A);
    if (dm_wait_reg(motor, 500) && motor->reg_value.u32 == target)
    {
        motor->native_mode = native_mode;
        DM4310_SendCommand(motor, 0xFC);   /* 使能 */
        osDelay(1);
        return;
    }

    /* 2. 失能 */
    DM4310_SendCommand(motor, 0xFD);
    osDelay(5);

    /* 3. 写 0x0A → 存 flash → 读回确认, 失败重试 3 次 */
    for (uint8_t retry = 0; retry < 3; retry++)
    {
        dm_write_reg(motor, 0x0A, target);
        osDelay(10);
        dm_save_flash(motor);
        osDelay(50);                        /* flash 写入最大约 30ms */

        dm_read_reg(motor, 0x0A);
        if (dm_wait_reg(motor, 100) && motor->reg_value.u32 == target)
            break;
    }

    /* 4. 使能 */
    DM4310_SendCommand(motor, 0xFC);
    osDelay(5);

    motor->native_mode = native_mode;
}
