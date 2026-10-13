/**
 * @file loader.h
 * @brief 拨盘 M2006 控制(移植自 Infantry-new shoot.c 的拨盘部分)
 */
#ifndef INF_LOADER_H
#define INF_LOADER_H

void LoaderInit(void);

/* 周期调用(100Hz): 拨轮连发 + 卡弹回转保护 */
void LoaderTask(void);

#endif /* INF_LOADER_H */
