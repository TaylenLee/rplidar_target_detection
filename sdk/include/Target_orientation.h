/****************************Copyright (c)**************************************************
**                            UESTC---FMD
**-------------------------------------File Info--------------------------------------------
** File Name:               Target_orientation.h
** Last modified Date:      2019.6.27
** Last Version:            V1.0.0
** Description:
**------------------------------------------------------------------------------------------
** Created By:              Taylen 李龙基
** Created date:            2019.6.27
** Version:                 V1.0.0
** Descriptions:
********************************************************************************************/
#pragma once
/*
**********************************************************************************************************
INCLUDES
**********************************************************************************************************
*/
#include "Target_orientation.h"
//using namespace std;
/*
**********************************************************************************************************
GLOBAL VARIABLES
**********************************************************************************************************
*/
extern int Sector_One_Upper_Limit_Idex;
extern int Sector_Two_Lower_Limit_Idex;
extern int Sector_Total_Num;
extern int Crop_Sector_Total_Num;
/*
**********************************************************************************************************
MACROS
**********************************************************************************************************
*/

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif
/*
选取是laser 正前方的扇型数据
扇区1：0°  ~ 60°
扇区2：300°~ 360°
*/
#define    Sector_One_Upper_Limit   30
#define    Sector_Two_Lower_Limit   330

#ifndef _count_of
#define _count_of(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif
/*
**********************************************************************************************************
TYPEDEFS
**********************************************************************************************************
*/

struct point_t {
	double x, y;
};

typedef struct _target_information_t {
	double target_slope;  // 相对于雷达坐标的倾角
	double target_length; //目标物长度
	double distance;      // 目标中心相到雷达坐标系距离雷达中心点的距离
} __attribute__((packed)) target_information_t,*target_information_p;
/*
typedef struct _target_information_transformed {
	_u8 s16_target_slope[2];  // 相对于雷达坐标的倾角
	_u8 s16_target_length[2]; //目标物长度
	_u8 s16_distance[2];      // 目标中心相到雷达坐标系距离雷达中心点的距离
} __attribute__((packed)) u8_target_information_t,*u8_target_information_p;
 */
typedef struct _modify_rplidar_response_measurement_node_t {
    _u16   angle_q6_checkbit; // check_bit:1;angle_q6:15;
    _u16   distance_q2;
} __attribute__((packed)) modify_rplidar_response_measurement_node_t;

/*
**********************************************************************************************************
FUNCTIONS
**********************************************************************************************************
*/
extern double distance_sqr(struct point_t const*a, struct point_t const*b);
extern double LeastSquaresFitting(point_t Laser_Data[], int Data_Len);
extern void Obtaining_Index_Boundary_SectorArea(rplidar_response_measurement_node_t Nodes[], int Nodes_Len);
extern void Crop_ScanData(rplidar_response_measurement_node_t Nodes[], int Nodes_Len, modify_rplidar_response_measurement_node_t Modify_Nodes[]);


