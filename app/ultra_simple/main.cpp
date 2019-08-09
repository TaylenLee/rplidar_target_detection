/*
 *  RPLIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


//user header files
#include "global.h"


//#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
using namespace std;

using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, const char * argv[]) {
    
    const char * opt_com_path = NULL;
    _u32         baudrateArray[2] = {115200, 256000};
    _u32         opt_com_baudrate = 0;
    u_result     op_result;

	//double target_slope = 0;
	//double target_length = 0
	target_information_t target_information;

    bool useArgcBaudrate = false;
	/*
    printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
           "Version: "RPLIDAR_SDK_VERSION"\n");
	*/
	printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
		"Version: %s\n", RPLIDAR_SDK_VERSION);
    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>2)
    {
        opt_com_baudrate = strtoul(argv[2], NULL, 10);
        useArgcBaudrate = true;
    }

    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#else
        opt_com_path = "/dev/rplidar";
#endif
    }

    // create the driver instance
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }
    
    rplidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    // make connection...
    if(useArgcBaudrate)
    {
        if(!drv)
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
        {
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_OK(op_result)) 
            {
                connectSuccess = true;
            }
            else
            {
                delete drv;
                drv = NULL;
            }
        }
    }
    else
    {
        size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
        for(size_t i = 0; i < baudRateArraySize; ++i)
        {
            if(!drv)
                drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
            if(IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
            {
                op_result = drv->getDeviceInfo(devinfo);

                if (IS_OK(op_result)) 
                {
                    connectSuccess = true;
                    break;
                }
                else
                {
                    delete drv;
                    drv = NULL;
                }
            }
        }
    }
    if (!connectSuccess) {
        
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        goto on_finished;
    }

    signal(SIGINT, ctrlc);
    
    drv->startMotor();
    // start scan...
    drv->startScan(0,1);
	delay(10);
    
    serial_initial();
    serial_open();

    // fetech result and print it out...
    while (1) {
        rplidar_response_measurement_node_t nodes[8192];
		modify_rplidar_response_measurement_node_t modify_nodes[500];
        size_t   count = _count_of(nodes);
		//printf("count is :%d \n", count);
        op_result = drv->grabScanData(nodes, count);
		//printf("count is :%d \n",count);
		
        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
			//printf("count is %d\n", count);
			Obtaining_Index_Boundary_SectorArea(nodes, count);
			//printf("Sector_One_Upper_Limit_Idex is %d\n", Sector_One_Upper_Limit_Idex);
			//printf("Sector_Two_Lower_Limit_Idex is %d\n", Sector_Two_Lower_Limit_Idex);
			//modify_rplidar_response_measurement_node_t modify_nodes[Sector_Total_Num];
			Crop_ScanData(nodes, count, modify_nodes);

			
			//打印剪切后的数据
			/*
			printf("start\n");
			for (int pos = 0; pos < Crop_Sector_Total_Num; ++pos) {
				printf("theta: %03.2f Dist: %04.2f \n",
					(modify_nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
					modify_nodes[pos].distance_q2 / 4.0f);

			}
			*/
			/*
            for (int pos = 0; pos < (int)count ; ++pos) {
                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
                    (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                    nodes[pos].distance_q2/4.0f,
                    nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
				
            }
			*/

			//获取扇形区目标边沿的点在雷达坐标系中的坐标（x,y）
			vector <point_t> target_points(Crop_Sector_Total_Num);
			//printf("new data begin\n");
			for (int i = 0; i < Crop_Sector_Total_Num; i++)
			{
				target_points[i].x = (modify_nodes[i].distance_q2 / 4.0f) * sin((modify_nodes[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f / 180 * M_PI);
				target_points[i].y = (modify_nodes[i].distance_q2 / 4.0f) * cos((modify_nodes[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f / 180 * M_PI);
				//printf("x: %05.2f y:%05.2f\n", target_points[i].x, target_points[i].y);
			}
			point_t *buffer = new point_t[target_points.size()];
			if (!target_points.empty())
			{
				memcpy(buffer, &target_points[0], target_points.size() * sizeof(point_t));
				/*
				printf("new data begin\n");
				for (int i = 0; i < Crop_Sector_Total_Num; i++)
				{
					printf("x: %08.2f y:%08.2f\n", buffer[i].x, buffer[i].y);
				}
				*/
					
			}
			int target_center_index = (floor(Crop_Sector_Total_Num / 2));
			target_information.distance = modify_nodes[target_center_index].distance_q2 / 4.0f /1000;

			target_information.target_slope = LeastSquaresFitting(buffer, Crop_Sector_Total_Num);
			//printf("target_slope is  %f \n", target_information.target_slope);

			target_information.target_length = distance_sqr(buffer, &buffer[Crop_Sector_Total_Num-1]);
			//printf("target length is  %f \n", target_information.target_length);

			//printf("distace: %f slope: %f length: %f\n", target_information.distance, target_information.target_slope, target_information.target_length);
            serial_send(&target_information);
			delete[]buffer;
        }
		
        if (ctrl_c_pressed){ 
            break;
        }
    }

    drv->stop();
    drv->stopMotor();
    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return 0;
}



