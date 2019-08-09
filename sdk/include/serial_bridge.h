/****************************Copyright (c)**************************************************
**                            UESTC---FMD
**-------------------------------------File Info--------------------------------------------
** File Name:               serial_bridge.h
** Last modified Date:      2019.8.1
** Last Version:            V1.0.0
** Description:
**------------------------------------------------------------------------------------------
** Created By:              Taylen 李龙基
** Created date:            2019.8.1
** Version:                 V1.0.0
** Descriptions:
********************************************************************************************/
#pragma once

#include "global.h"

extern void serial_initial();
extern void serial_open();
extern void serial_recv(uint8_t *recvBuffer,size_t recvSize);
extern void serial_send(target_information_p target_msg);
