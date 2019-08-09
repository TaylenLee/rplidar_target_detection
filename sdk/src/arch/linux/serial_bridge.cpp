/****************************Copyright (c)**************************************************
**                            UESTC---FMD                  
**-------------------------------------File Info--------------------------------------------
** File Name:               serial_bridge.cpp
** Last modified Date:      2019.8.1
** Last Version:            V1.0.0
** Description:
**------------------------------------------------------------------------------------------
** Created By:              Taylen 李龙基
** Created date:            2019.8.1
** Version:                 V1.0.0
** Descriptions:
********************************************************************************************/

#include "global.h"
/*searial */
std::string param_port_path_;
int param_baudrate_;
rp::hal::serial_rxtx* _rxtx =NULL;

void serial_initial()
{
    param_port_path_ = "/dev/CH340";
    param_baudrate_  = 115200;
}

void serial_open()
{
    int i=10;
    if(_rxtx == NULL)
    {
        _rxtx = rp::hal::serial_rxtx::CreateRxTx();
    }
    while(i--)
    {
        if (!_rxtx->bind(param_port_path_.c_str(),param_baudrate_) || !_rxtx->open())
        {
            printf("%s open ERROR",param_port_path_.c_str());
            delay(10);
            continue;
        }
        _rxtx->flush(0);
        printf("%s open OK,baudrate %d",param_port_path_.c_str(),param_baudrate_);
        return;
    }
    exit(0);
}

void serial_recv(uint8_t *recvBuffer,size_t recvSize)
{
    do
    {
       do
       {
           recvBuffer[0] = 0;
           if(rp::hal::serial_rxtx::ANS_OK != _rxtx->waitfordata(1,-1,&recvSize))
           {
               printf("serial_rx waitfordata _PROTOCAL_SYN_ERROR");
               exit(0);
               continue;
           }
           _rxtx->recvdata(&recvBuffer[0],1);

           if(recvBuffer[0]!= _SERIAL_SYN_CODE_START)
           {
               printf("serial_rx _PROTOCOL_SYN_START");
               continue;
           }
           break;
       } while (1);
       
       if(rp::hal::serial_rxtx::ANS_OK != _rxtx->waitfordata(1,-1,&recvSize))
       {
           printf("serial_rx waitfordata _PROTOCAL_SYN_ERROR");
           exit(0);
           continue;
       }

       _rxtx->recvdata(&recvBuffer[1],sizeof(serialDsta) - 1);
       if(recvBuffer[sizeof(serialDsta) - 2] != '\r' && recvBuffer[sizeof(serialDsta)- 2] != '\n')
       {
           printf("serial_rx DATA_ERROR");
           continue;
       }
       return;
    } while (1);
    
}

void serial_send(target_information_p target_msg)
{
    serialDsta package_data;
    _u16 modify_distance,modify_targetlength;
    _s16 modify_slope;
    modify_distance = (_u16)((target_msg->distance) * 100);
    modify_targetlength = (_u16)((target_msg->target_length) * 100);
    modify_slope = (_s16)((target_msg->target_slope) * 100);
    package_data.syn = _SERIAL_SYN_CODE_START;
    package_data.type = SMALL_BRIKE;

    package_data.buf[0] = LowByte(modify_distance);
    //printf("LowByte(modify_distance) is %d\n",package_data.buf[0]);
    package_data.buf[1] = HighByte(modify_distance);
    package_data.buf[2] = LowByte(modify_targetlength);
    package_data.buf[3] = HighByte(modify_targetlength);

    package_data.buf[4] = LowByte(modify_slope);
    package_data.buf[5] = HighByte(modify_slope);
    
    package_data.syn_CR = '\r';
    package_data.syn_LF = '\n';
    _rxtx->senddata((uint8_t*)&package_data,sizeof(serialDsta));
}
