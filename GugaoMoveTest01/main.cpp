#include<stdio.h>
#include "gts.h"
#include "conio.h"
#include "math.h"
#define CORE 1		// 控制卡内核序号
#define AXIS 1		// 定义轴号

// 该函数检测某条GTN指令的执行结果，command为指令名称，error为指令执行返回值
void commandhandler(char* command, short error)
{
    // 如果指令执行返回值为非0，说明指令执行错误，向屏幕输出错误结果
    if (error)
    {
        printf("%s = %d\n", command, error);
    }
}

int main()
{
    short sRtn;
    short sEcatSts;
    TTrapPrm trap;
    long lAxisSts;
    double prfPos;
    short setTorque=600;
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
    // AXIS轴规划位置清零
    sRtn = GTN_SetPrfPos(CORE, AXIS, 0);
    commandhandler((char*)"GTN_SetPrfPos", sRtn);
    // 将AXIS轴设为点位模式
    sRtn = GTN_PrfTrap(CORE, AXIS);
    commandhandler((char*)"GTN_PrfTrap", sRtn);
    // 读取点位运动参数
    sRtn = GTN_GetTrapPrm(CORE, AXIS, &trap);
    commandhandler((char*)"GTN_GetTrapPrm", sRtn);
    trap.acc = 0.25;
    trap.dec = 0.125;
    trap.smoothTime = 25;
    // 设置点位运动参数
    sRtn = GTN_SetTrapPrm(CORE, AXIS, &trap);
    commandhandler((char*)"GTN_SetTrapPrm", sRtn);
    // 设置AXIS轴的目标位置
    sRtn = GTN_SetPos(CORE, AXIS, 5000L);
    commandhandler((char*)"GTN_SetPos", sRtn);
    // 设置AXIS轴的目标速度
    sRtn = GTN_SetVel(CORE, AXIS, 10);
    commandhandler((char*)"GTN_SetVel", sRtn);
     
    sRtn = GTN_SetEcatAxisPT(CORE, AXIS, 100);
    commandhandler((char*)"GTN_SetEcatAxisPT", sRtn);
    // 启动AXIS轴的运动
    sRtn = GTN_Update(CORE, 1 << (AXIS - 1));
    commandhandler((char*)"GTN_Update", sRtn);
    do
    {
        // 读取AXIS轴的状态
        sRtn = GTN_GetSts(CORE, AXIS, &lAxisSts);
        commandhandler((char*)"GTN_GetSts", sRtn);
        // 读取AXIS轴的规划位置
        sRtn = GTN_GetPrfPos(CORE, AXIS, &prfPos);
        commandhandler((char*)"GTN_GetPrfPos", sRtn);
        short torque=0;
        GTN_GetEcatAxisAtlTorque(CORE, AXIS, &torque);


        short getAxis=0;
        GTN_GetEcatRawData(CORE,11,2,(unsigned char*)&getAxis);
        printf("getAxis=%d\n", getAxis);

        printf("lAxisSts=0x%-10lxprfPos=%-10.1lf_torque=%d\n", lAxisSts, prfPos,torque);
        if (abs(torque) > setTorque)
        {
            printf("torque=%d over set %d\n",  torque,setTorque);
            GTN_Stop(CORE, AXIS, 1);
            break;
        }
    } while (lAxisSts & 0x400);	// 等待AXIS轴规划停止


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

