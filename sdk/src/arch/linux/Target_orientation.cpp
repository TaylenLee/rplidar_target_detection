/****************************Copyright (c)**************************************************
**                            UESTC---FMD                  
**-------------------------------------File Info--------------------------------------------
** File Name:               Target_orientation.cpp
** Last modified Date:      2019.6.27
** Last Version:            V1.0.0
** Description:
**------------------------------------------------------------------------------------------
** Created By:              Taylen 李龙基
** Created date:            2019.6.27
** Version:                 V1.0.0
** Descriptions:
********************************************************************************************/
/*
**********************************************************************************************************
INCLUDES
**********************************************************************************************************
*/
#include "global.h"
/*
**********************************************************************************************************
CONSTANTS
**********************************************************************************************************
* /

/*
**********************************************************************************************************
GLOBAL VARIABLES
**********************************************************************************************************
*/
int Sector_One_Upper_Limit_Idex = 0;
int Sector_Two_Lower_Limit_Idex = 0;
int Sector_Total_Num = 1;
int Crop_Sector_Total_Num = 0;
/*
**********************************************************************************************************
FUNCTIONS
**********************************************************************************************************
*/

double distance_sqr(struct point_t const*a, struct point_t const*b)
{
	double ax, ay, bx, by;
	ax = a->x /1000.0;
	bx = b->x/ 1000.0;

	ay = a->y /1000.0;
	by = b->y / 1000.0;


	return sqrtf((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
}

/*
函 数 名：float LeastSquaresFitting
函数功能：最小二乘法拟合直线
输入参数：砖块边缘点位于雷达坐标中的坐标数组:point_t Laser_Data[]
输出参数：砖块边缘的斜率:slope
*/
double LeastSquaresFitting(point_t Laser_Data[],int Data_Len)
//float LeastSquaresFitting(vector <point_t> Laser_Data(), int Data_Len)
{
	double slope; //拟合直线的斜率
	double av_x, av_y;
	double L_xx, L_xy;
	//int Data_Len = _count_of(Laser_Data);
	av_x = 0; //X的平均值
	av_y = 0; //Y的平均值
	L_xx = 0; 
	L_xy = 0; 
	int i = 0;
	for (i = 0; i < Data_Len; i++) //计算X、Y的平均值
	{
		av_x += Laser_Data[i].x;
		av_y += Laser_Data[i].y;
	}
	av_x = av_x / Data_Len;
	av_y = av_y / Data_Len;

	for (i = 0; i < Data_Len; i++) //计算Lxx、Lyy和Lxy
	{
		L_xx += (Laser_Data[i].x - av_x)*(Laser_Data[i].x - av_x);
		L_xy += (Laser_Data[i].x - av_x)*(Laser_Data[i].y - av_y);
	}

	slope = L_xy / L_xx; //斜率
	return slope;
}

/*
函 数 名：Obtaining_Index_Boundary_SectorArea
函数功能：获得感兴趣扫描区域边界数据索引
输入参数：按角度递增排序的扫描数据:Nodes[];数据长度：Nodes_Len
输出参数：void
*/
void Obtaining_Index_Boundary_SectorArea(rplidar_response_measurement_node_t Nodes[], int Nodes_Len)
{
	double previous_angle,next_angle;
	previous_angle = (Nodes[0].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
	for (int pos = 1; pos < (int)Nodes_Len; ++pos) 
	{
		next_angle = (Nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
		if (previous_angle <= Sector_One_Upper_Limit && Sector_One_Upper_Limit <= next_angle)
		{
			Sector_One_Upper_Limit_Idex = pos - 1;
		}
		if (previous_angle <= Sector_Two_Lower_Limit && Sector_Two_Lower_Limit <= next_angle)
		{
			Sector_Two_Lower_Limit_Idex = pos;
			Sector_Total_Num = Nodes_Len - Sector_Two_Lower_Limit_Idex + Sector_One_Upper_Limit_Idex;
			break;
		}
		previous_angle = next_angle;
	}

}

/*
函 数 名：
函数功能：过滤rplidar的扫描数据
输入参数：
输出参数：
*/


/*
函 数 名：Crop_ScanData
函数功能：裁剪rplidar的扫描数据
输入参数：按角度递增排序的扫描数据:Nodes[];数据长度：Nodes_Len；剪裁后的扫描数据指针
输出参数：void
*/
void Crop_ScanData(rplidar_response_measurement_node_t Nodes[], int Nodes_Len, modify_rplidar_response_measurement_node_t Modify_Nodes[])
{
	int i = 0, j = 0;
	for (i = Sector_Two_Lower_Limit_Idex; i < Nodes_Len; i++)
	{
		if (Nodes[i].distance_q2 != 0)
		{
			Modify_Nodes[j].angle_q6_checkbit = Nodes[i].angle_q6_checkbit;
			Modify_Nodes[j].distance_q2 = Nodes[i].distance_q2;
			j += 1;
		}
	}
	
	for (i = 0; i < Sector_One_Upper_Limit_Idex; i++)
	{
		if (Nodes[i].distance_q2 != 0)
		{
			Modify_Nodes[j].angle_q6_checkbit = Nodes[i].angle_q6_checkbit;
			Modify_Nodes[j].distance_q2 = Nodes[i].distance_q2;
			j += 1;
		}
	}

	Crop_Sector_Total_Num = j;
}

/*
函 数 名：
函数功能：获取扇形区目标边沿的点在雷达坐标系中的坐标（x,y）
输入参数：
输出参数：void
*/

void Get_Target_Point_Coordinate(modify_rplidar_response_measurement_node_t Modify_Nodes[], int Modify_Nodes_Len)
{
	for (int i = 0; i < Crop_Sector_Total_Num; i++)
	{
		;//target_points[i]
	}
}
