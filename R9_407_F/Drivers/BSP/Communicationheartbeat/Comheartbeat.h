/*
 * @Author: lisir lisir@rehand.com
 * @Date: 2024-08-12 14:01:53
 * @LastEditors: lisir lisir@rehand.com
 * @LastEditTime: 2024-08-14 18:06:57
 * @FilePath: \R9_407F\R9_407F\R9_407_F\Drivers\BSP\Communicationheartbeat\Comheartbeat.h
 * @Description: 
 * 
 * Copyright (c) 2024-2034  , Rehand Medical Technology Co. LTDl, All Rights Reserved. 
 */
#ifndef HEART_H
#define HEART_H
#include "./SYSTEM/sys/sys.h"
#include "./BSP/R9/Slavemodbus.h"

typedef enum 
{
    Fail ,
    Success
}E_COMSTATE;

typedef struct 
{
    uint16_t  detect_time; /*检测计时 该设置需大于 200ms*/
    uint8_t detect_falge; /*需检测标志位*/
    E_COMSTATE com_state;
}STRUCT_COMHEART;

extern STRUCT_COMHEART comheartstate;
void  ComheartReset(void);
uint8_t  ComheartDetect(uint16_t overtcont);

#endif
