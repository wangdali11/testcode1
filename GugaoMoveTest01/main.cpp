#include<stdio.h>
#include "gts.h"
#include "conio.h"
#include "math.h"
#define CORE 1		// 控制卡内核序号
#define AXIS 1		// 定义轴号
#include <iostream>
#include <chrono>
#include <thread>

// 该函数检测某条GTN指令的执行结果，command为指令名称，error为指令执行返回值
void commandhandler(char* command, short error)
{
    // 如果指令执行返回值为非0，说明指令执行错误，向屏幕输出错误结果
    if (error)
    {
        printf("%s = %d\n", command, error);
    }
}

/*
力矩控制
设定力的方向后，就会决定是正转还是反转。
到达指定设定力矩后，不会停止，只是输出的力不会去超过这个力矩，需要你手动停止。
举个例子： 电机遇到障碍物后，力矩不会再增大，维持该力矩。障碍物移开，电机会继续运动。
运动速度由最大速度变量(0x6080)限制。这个变量也会影响其他模式，建议设置前读取一个该变量，存储一下。
*/



int main()
{
    short sRtn;
    short sEcatSts;
    TTrapPrm trap;
    long lAxisSts;
    double prfPos;
  
    // 打开运动控制器
    sRtn = GTN_Open();
    // 指令返回值检测
    commandhandler((char*)"GTN_Open", sRtn);
    if (sRtn)
    {
        printf("Failure to access cord!\n");
        return -1;
    }
    sRtn = GTN_InitEcatComm(CORE);
    commandhandler((char*)"GTN_InitEcatComm", sRtn);
    if (sRtn)
    {
        printf("EtherCAT communication error!\n");
        return -1;
    }
    do {//wait untill EtherCAT communication OK
        sRtn = GTN_IsEcatReady(CORE, &sEcatSts);
    } while (sEcatSts != 1 || sRtn != 0);
    // 开始EtherCAT通讯
    sRtn = GTN_StartEcatComm(CORE);
    commandhandler((char*)"GTN_StartEcatComm", sRtn);
    // 复位运动控制器
    sRtn = GTN_Reset(CORE);
    commandhandler((char*)"GTN_Reset", sRtn);
    // 清除各轴的报警和限位
    sRtn = GTN_ClrSts(CORE, AXIS, 8);
    commandhandler((char*)"GTN_ClrSts", sRtn);
    // 伺服使能
    sRtn = GTN_AxisOn(CORE, AXIS);
    commandhandler((char*)"GTN_AxisOn", sRtn);

    // 位置清零
    sRtn = GTN_ZeroPos(CORE, AXIS);
    commandhandler((char*)"GTN_ZeroPos", sRtn);

    uint32_t getSpeedMax;
    sRtn =GTN_GetEcatRawData(CORE, 9,4,(unsigned char*)&getSpeedMax);
    commandhandler((char*)"getSpeedMax", sRtn);
    std::cout << "getspeedmax__" <<getSpeedMax<< std::endl;

    // 最大速度限定 
    uint32_t speedmax=30;
    sRtn=GTN_SetEcatRawData(CORE, 9,4,(unsigned char*)&speedmax);
    commandhandler((char*)"GTN_SetEcatRawData speedmax", sRtn);

    // 力矩控制模式
    sRtn = GTN_SetEcatAxisMode(CORE, AXIS, 10);
    commandhandler((char*)"GTN_SetEcatAxisMode", sRtn);
    //  设置力矩
    short setTorque=300;
    sRtn = GTN_SetEcatAxisPT(CORE, AXIS, setTorque);
    commandhandler((char*)"GTN_SetEcatRawData setTorque", sRtn);
    while (1)
    {
        short torque=0;
        // 读力矩的
        GTN_GetEcatAxisAtlTorque(CORE, AXIS, &torque);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        printf("torque=%d\n", torque);
        if (abs(torque) >= abs(setTorque))
        {
            printf("torque=%d over set %d,stop\n",  torque,setTorque);
            GTN_Stop(CORE, AXIS, 1);
            break;
        }

    }
    // 伺服关闭
    sRtn = GTN_AxisOff(CORE, AXIS);
    commandhandler((char*)"GTN_AxisOff", sRtn);
    printf("\nPlease press anykey to continue!\n");
    _getch();
    // 关闭EtherCAT通讯
    sRtn = GTN_TerminateEcatComm(CORE);
    commandhandler((char*)"GTN_TerminateEcatComm", sRtn);
    // 关闭运动控制器
    sRtn = GTN_Close();
    commandhandler((char*)"GTN_Close", sRtn);
    return 0;
}

