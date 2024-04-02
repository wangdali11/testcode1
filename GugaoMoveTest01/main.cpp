#include<stdio.h>
#include "gts.h"
#include "conio.h"
#include "math.h"
#define CORE 1		// ���ƿ��ں����
#define AXIS 1		// �������
#include <iostream>
#include <chrono>
#include <thread>

// �ú������ĳ��GTNָ���ִ�н����commandΪָ�����ƣ�errorΪָ��ִ�з���ֵ
void commandhandler(char* command, short error)
{
    // ���ָ��ִ�з���ֵΪ��0��˵��ָ��ִ�д�������Ļ���������
    if (error)
    {
        printf("%s = %d\n", command, error);
    }
}
#include <iostream>
#include <fstream>
#include <ctime>
#include <cstdarg>
#include <iomanip>
void log(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto now_c = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_ms.time_since_epoch()) % 1000;

    tm ltm;
    localtime_s(&ltm, &now_c);

    std::ofstream file("a.txt", std::ios_base::app);
    file << "[" << 1900 + ltm.tm_year << "-" << 1 + ltm.tm_mon << "-" << ltm.tm_mday << " "
        << ltm.tm_hour << ":" << ltm.tm_min << ":" << ltm.tm_sec << "." << std::setfill('0') << std::setw(3) << ms.count() << "] " << buffer ;
    file.close();
}

int mainx() {
    log("This is a log message.");
    
    return 0;
}

/*
���ؿ���
�趨���ķ���󣬾ͻ��������ת���Ƿ�ת��
����ָ���趨���غ󣬲���ֹͣ��ֻ�������������ȥ����������أ���Ҫ���ֶ�ֹͣ��
�ٸ����ӣ� ��������ϰ�������ز���������ά�ָ����ء��ϰ����ƿ������������˶���
�˶��ٶ�������ٶȱ���(0x6080)���ơ��������Ҳ��Ӱ������ģʽ����������ǰ��ȡһ���ñ������洢һ�¡�
*/



int main()
{
    short sRtn;
    short sEcatSts;
    TTrapPrm trap;
    long lAxisSts;
  
  
    // ���˶�������
    sRtn = GTN_Open();
    // ָ���ֵ���
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
    // ��ʼEtherCATͨѶ
    sRtn = GTN_StartEcatComm(CORE);
    commandhandler((char*)"GTN_StartEcatComm", sRtn);
    // ��λ�˶�������
    sRtn = GTN_Reset(CORE);
    commandhandler((char*)"GTN_Reset", sRtn);
    // �������ı�������λ
    sRtn = GTN_ClrSts(CORE, AXIS, 8);
    commandhandler((char*)"GTN_ClrSts", sRtn);
    // �ŷ�ʹ��
    sRtn = GTN_AxisOn(CORE, AXIS);
    commandhandler((char*)"GTN_AxisOn", sRtn);

    // λ������
    sRtn = GTN_ZeroPos(CORE, AXIS);
    commandhandler((char*)"GTN_ZeroPos", sRtn);

    uint32_t getSpeedMax;
    sRtn =GTN_GetEcatRawData(CORE, 9,4,(unsigned char*)&getSpeedMax);
    commandhandler((char*)"getSpeedMax", sRtn);
    std::cout << "getspeedmax__" <<getSpeedMax<< std::endl;

    // ����ٶ��޶� 
    uint32_t speedmax=30;
    sRtn=GTN_SetEcatRawData(CORE, 9,4,(unsigned char*)&speedmax);
    commandhandler((char*)"GTN_SetEcatRawData speedmax", sRtn);

    // ���ؿ���ģʽ
    sRtn = GTN_SetEcatAxisMode(CORE, AXIS, 10);
    commandhandler((char*)"GTN_SetEcatAxisMode", sRtn);
    //  ��������
    short setTorque=-300;
    sRtn = GTN_SetEcatAxisPT(CORE, AXIS, setTorque);
    commandhandler((char*)"GTN_SetEcatRawData setTorque", sRtn);
    int i=0;
    
    while (1)
    {
        short torque=0;
        // �����ص�
        GTN_GetEcatAxisAtlTorque(CORE, AXIS, &torque);
        // ��ȡAXIS���λ��
        double prfPos;
        sRtn = GTN_GetAxisEncPos(CORE, AXIS, &prfPos);
        commandhandler((char*)"GTN_GetPrfPos", sRtn);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        printf("torque=%d_prfPos=%-10.1lf\n", torque,prfPos);
        log("torque=%d_prfPos=%-10.1lf\n", torque,prfPos);
        if (abs(torque) >= abs(setTorque))
        {
            if(i++ > 1000)
            {
                printf("torque=%d over set %d,stop\n",  torque,setTorque);
                GTN_Stop(CORE, AXIS, 1);
                break;
            }
            
        }
    }
    // �ŷ��ر�
    sRtn = GTN_AxisOff(CORE, AXIS);
    commandhandler((char*)"GTN_AxisOff", sRtn);
    printf("\nPlease press anykey to continue!\n");
    _getch();
    // �ر�EtherCATͨѶ
    sRtn = GTN_TerminateEcatComm(CORE);
    commandhandler((char*)"GTN_TerminateEcatComm", sRtn);
    // �ر��˶�������
    sRtn = GTN_Close();
    commandhandler((char*)"GTN_Close", sRtn);
    return 0;
}

