/****************************Copyright (c)**************************************************
**                            UESTC---FMD
**-------------------------------------File Info--------------------------------------------
** File Name:               usr_protocol.h
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

#define _SERIAL_SYN_CODE_START 0xFA

#define BIG_BRIKE    0XA1
#define MIDDLE_BRIKE 0XA2
#define SMALL_BRIKE    0XA3


#define LowByte(w)  ((_u8)((0xff)&(w)))
#define HighByte(w) ((_u8)((0xff)&((w) >> 8)))

#pragma pack(1)
typedef struct _searial_data
{
    _u8 syn;
    _u8 type;
    /* 
    union
    {
        target_information_t target_information_l;
        double buf[3];
    }data;
    */
    _u8 buf[6];
    
    _u8 syn_CR;
    _u8 syn_LF;
}serialDsta;
#pragma pack()