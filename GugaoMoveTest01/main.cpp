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

int main()
{
    short sRtn;
    short sEcatSts;
    TTrapPrm trap;
    long lAxisSts;
    double prfPos;
    short setTorque=6000;
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
    // // AXIS��滮λ������
    // sRtn = GTN_SetPrfPos(CORE, AXIS, 0);
    // commandhandler((char*)"GTN_SetPrfPos", sRtn);
    // // ��AXIS����Ϊ��λģʽ
    // sRtn = GTN_PrfTrap(CORE, AXIS);
    // commandhandler((char*)"GTN_PrfTrap", sRtn);
    // // ��ȡ��λ�˶�����
    // sRtn = GTN_GetTrapPrm(CORE, AXIS, &trap);
    // commandhandler((char*)"GTN_GetTrapPrm", sRtn);
    // trap.acc = 0.25;
    // trap.dec = 0.125;
    // trap.smoothTime = 25;
    // // ���õ�λ�˶�����
    // sRtn = GTN_SetTrapPrm(CORE, AXIS, &trap);
    // commandhandler((char*)"GTN_SetTrapPrm", sRtn);
    // // ����AXIS���Ŀ��λ��
    // sRtn = GTN_SetPos(CORE, AXIS, -115000L);
    // commandhandler((char*)"GTN_SetPos", sRtn);
    // // ����AXIS���Ŀ���ٶ�
    // sRtn = GTN_SetVel(CORE, AXIS, 10);
    // commandhandler((char*)"GTN_SetVel", sRtn);


 uint32_t getspeedmax;
    GTN_GetEcatRawData(CORE, 9,4,(unsigned char*)&getspeedmax);
    commandhandler((char*)"getspeedmax", sRtn);
    std::cout << "getspeedmax__" <<getspeedmax<< std::endl;



    // unsigned  char stoptorque[10] = { 0 };
    // stoptorque[1] = -100;
    uint32_t speedmax=60;
    GTN_SetEcatRawData(CORE, 9,4,(unsigned char*)&speedmax);
    commandhandler((char*)"GTN_SetEcatRawData", sRtn);


    GTN_GetEcatRawData(CORE, 9,4,(unsigned char*)&getspeedmax);
    commandhandler((char*)"getspeedmax", sRtn);
    std::cout << "getspeedmax__" <<getspeedmax<< std::endl;



   sRtn = GTN_SetEcatAxisMode(CORE, AXIS, 10);
    commandhandler((char*)"GTN_SetEcatAxisMode", sRtn);
    //  ��������
    sRtn = GTN_SetEcatAxisPT(CORE, AXIS, -300);
    commandhandler((char*)"GTN_SetEcatAxisPT", sRtn);



    

    
    while (1)
    {
        short torque=0;
        // �����ص�
        GTN_GetEcatAxisAtlTorque(CORE, AXIS, &torque);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        printf("torque=%d\n", torque);

    }
    // GTN_Stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "Delay complete!" << std::endl;
   

    // // ����AXIS����˶�
    // sRtn = GTN_Update(CORE, 1 << (AXIS - 1));
    // commandhandler((char*)"GTN_Update", sRtn);
    
    // int i = 0;
    // do
    // {
    //     // ��ȡAXIS���״̬
    //     sRtn = GTN_GetSts(CORE, AXIS, &lAxisSts);
    //     commandhandler((char*)"GTN_GetSts", sRtn);
    //     // ��ȡAXIS��Ĺ滮λ��
    //     sRtn = GTN_GetPrfPos(CORE, AXIS, &prfPos);
    //     commandhandler((char*)"GTN_GetPrfPos", sRtn);
    //     short torque=0;

    //     // �����ص�
    //     GTN_GetEcatAxisAtlTorque(CORE, AXIS, &torque);


    //     // unsigned char getAxis[10] = {0};
    //     // sRtn = GTN_GetEcatRawData(CORE, 5,4,(unsigned char*)&getAxis);
    //     // commandhandler((char*)"GTN_GetPrfPos", sRtn);
    //     // printf("getAxis=%d_%d_%d_%d\n", getAxis[0], getAxis[1],getAxis[2], getAxis[3]);
    //     // short getAxis = 0;
    //     // sRtn = GTN_GetEcatRawData(CORE, 5,2,(unsigned char*)&getAxis);
    //     // printf("getAxis=%d\n", getAxis);
    //     //printf("getAxis=%d_%d\n", getAxis[0], getAxis[1]);

    //     printf("lAxisSts=0x%-10lxprfPos=%-10.1lf_torque=%d\n", lAxisSts, prfPos,torque);
    //     if (abs(torque) > setTorque)
    //     {
    //         printf("torque=%d over set %d\n",  torque,setTorque);
    //         GTN_Stop(CORE, AXIS, 1);
    //         break;
    //     }
    // } while (i++<10000);


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

