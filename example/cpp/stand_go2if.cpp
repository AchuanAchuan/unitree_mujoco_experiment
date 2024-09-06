#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

#include "common/unitreeLeg.h"
using namespace std;

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

class Custom
{
public:
    Custom(){};
    ~Custom(){};
    void Init();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void *messages);
    void LowCmdWrite();

private:
    double stand_up_joint_pos[12];
    double stand_down_joint_pos[12];
    double dt = 0.002;
    double runing_time = 0.0;
    double phase = 0.0;

    QuadrupedLeg* _Legs[4];
    Vec34 qLegs;
    float Pz = 0;
    Vec3 pDesRef;
    Vec3 pDes;
    Vec3 qLegsdes;
    Vec3 qLegsdesRef;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};     // default init
    unitree_go::msg::dds_::LowState_ low_state{}; // default init

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;
};

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;         
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void Custom::Init()
{
    InitLowCmd();
    /*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

    /*loop publishing thread*/
    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, int(dt * 1000000), &Custom::LowCmdWrite, this);
}

void Custom::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01); // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }

    _Legs[0] = new Go2Leg(0, Vec3( 0.1934, -0.0465, 0));
    _Legs[1] = new Go2Leg(1, Vec3( 0.1934,  0.0465, 0));
    _Legs[2] = new Go2Leg(2, Vec3(-0.1934, -0.0465, 0));
    _Legs[3] = new Go2Leg(3, Vec3(-0.1934,  0.0465, 0));

    pDesRef[0] = 0.0;
    pDesRef[1] = -0.0885;
    pDesRef[2] = -0.14999;
    qLegsdesRef = _Legs[0]->calcQ(pDesRef , FrameType::HIP);
    std::cout << "qLegsdesRef:" << qLegsdesRef[0] <<" "<< qLegsdesRef[1] <<" "<< qLegsdesRef[2] << endl;

    std::cout << "inputS:Pz"<< endl;
    std::cin >> Pz; 
    pDes[0] = 0.0;
    pDes[1] = -0.0935;
    pDes[2] = -Pz;
    qLegsdes = _Legs[0]->calcQ(pDes , FrameType::HIP);
    std::cout << "qLeg0des:" << qLegsdes[0] <<" "<< qLegsdes[1] <<" "<< qLegsdes[2] <<endl;

    for(int i = 0;i < 12;i++){
        if(i == 3 || i == 9){
            stand_down_joint_pos[i] = -qLegsdesRef[i%3];
        }
        else{
            stand_down_joint_pos[i] = qLegsdesRef[i%3];     
        }
    }
    for(int i = 0;i < 12;i++){
        if(i == 3 || i == 9){
            stand_up_joint_pos[i] = -qLegsdes[i%3];
        }
        else{
            stand_up_joint_pos[i] = qLegsdes[i%3];     
        }
    }
    // stand_down_joint_pos[12] = {qLegsdesRef[0], qLegsdesRef[1], qLegsdesRef[2], -qLegsdesRef[0], qLegsdesRef[1], qLegsdesRef[2],
    //                                  qLegsdesRef[0], qLegsdesRef[1], qLegsdesRef[2], -qLegsdesRef[0], qLegsdesRef[1], qLegsdesRef[2]};
    // stand_up_joint_pos[12] = {qLegsdes[0], qLegsdes[1], qLegsdes[2], -qLegsdes[0], qLegsdes[1], qLegsdes[2],
    //                                  qLegsdes[0], qLegsdes[1], qLegsdes[2], -qLegsdes[0], qLegsdes[1], qLegsdes[2]};
}

void Custom::LowStateMessageHandler(const void *message)
{
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;
}

void Custom::LowCmdWrite()
{

    runing_time += dt;
    if (runing_time < 3.0)
    {
        // Stand up in first 3 second

        // Total time for standing up or standing down is about 1.2s
        phase = tanh(runing_time / 1.2);
        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd()[i].q() = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i];
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = phase * 50.0 + (1 - phase) * 20.0;
            low_cmd.motor_cmd()[i].kd() = 3.5;
            low_cmd.motor_cmd()[i].tau() = 0;
        }
    }
    else
    {
        // Then stand down
        phase = tanh((runing_time - 3.0) / 1.2);
        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd()[i].q() = phase * stand_down_joint_pos[i] + (1 - phase) * stand_up_joint_pos[i];
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = 50;
            low_cmd.motor_cmd()[i].kd() = 3.5;
            low_cmd.motor_cmd()[i].tau() = 0;
        }
    }

    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);
}

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(1, "lo");
    }
    else
    {
        ChannelFactory::Instance()->Init(0, argv[1]);
    }
    std::cout << "Press enter to start";
    std::cin.get();
    Custom custom;
    custom.Init();

    while (1)
    {
        sleep(10);
    }

    return 0;
}
