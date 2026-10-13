/**
 * @file robot_task.h
 * @brief FreeRTOS 任务初始化与各任务入口
 *
 * 仅被 robot.c 包含一次(函数在头文件内定义)。
 */
#ifndef INF_ROBOT_TASK_H
#define INF_ROBOT_TASK_H

#include "cmsis_os.h"
#include "robot.h"
#include "daemon.h"
#include "dmmotor.h"
#include "motor_task.h"

osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;

void StartROBOTTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);

/* 创建所有 FreeRTOS 任务(在 RobotInit 里调用, 需在所有 DMMotorInit 之后) */
void OSTaskInit(void)
{
    osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 512);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

    /* DM 电机各自起独立任务(需在所有 DMMotorInit 之后调用) */
    DMMotorControlInit();
}

void StartROBOTTASK(void const *argument)
{
    (void)argument;
    for (;;)
    {
        RobotTask();          /* 100Hz: remote → gimbal(pitch) / loader */
        osDelay(10);
    }
}

void StartMOTORTASK(void const *argument)
{
    (void)argument;
    for (;;)
    {
        MotorControlTask();   /* 1kHz: DJI 电机闭环(M2006 拨盘) */
        osDelay(1);
    }
}

void StartDAEMONTASK(void const *argument)
{
    (void)argument;
    for (;;)
    {
        DaemonTask();         /* 100Hz */
        osDelay(10);
    }
}

#endif /* INF_ROBOT_TASK_H */
