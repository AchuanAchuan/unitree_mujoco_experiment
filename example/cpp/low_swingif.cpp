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
//#include "common/unitreeRobot.h"
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
    explicit Custom()
    {}

    ~Custom()
    {}

    void Init();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void* messages);
    void LowCmdWrite();

private:
    float qInit[3] = {0};
    float qDes[3] = {0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};
    float Kd[3] = {0};
    double time_consume = 0;
    int rate_count1 = 0;
    int rate_count2 = 0;
    int rate_count3 = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002; // 0.001~0.01

    QuadrupedLeg* _Legs[4];
    Vec34 qLegs;
    float Pz = 0;
    Vec3 pDesRef;
    Vec3 pDes;
    Vec3 qLegsdes;
    Vec3 qLegsdesRef;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
    unitree_go::msg::dds_::LowState_ low_state{};  // default init

    

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;
};

uint32_t crc32_core(uint32_t* ptr, uint32_t len)
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
    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, 2000, &Custom::LowCmdWrite, this);
}

void Custom::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for(int i=0; i<20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
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
    pDesRef[1] = -0.09;
    pDesRef[2] = -0.15;
    qLegsdesRef = _Legs[0]->calcQ(pDesRef , FrameType::HIP);
    std::cout << "qLegsdesRef:" << qLegsdesRef[0] <<" "<< qLegsdesRef[1] <<" "<< qLegsdesRef[2] <<endl;

    std::cout << "inputS:Pz"<< endl;
    std::cin >> Pz; 
    pDes[0] = 0.0;
    pDes[1] = -0.09;
    pDes[2] = -Pz;
    qLegsdes = _Legs[0]->calcQ(pDes , FrameType::HIP);
    std::cout << "qLeg0des:" << qLegsdes[0] <<" "<< qLegsdes[1] <<" "<< qLegsdes[2] <<endl;
}

void Custom::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos * (1 - rate) + targetPos * rate;
    return p;
}

void Custom::LowCmdWrite()
{
    motiontime++;

    if (motiontime >= 0)
    {
        // first, get record initial position
        if (motiontime >= 0 && motiontime < 20)
        {
            qInit[0] = low_state.motor_state()[0].q();
            qInit[1] = low_state.motor_state()[1].q();
            qInit[2] = low_state.motor_state()[2].q();
        }
        // second, move to the origin point of a sine movement with Kp Kd
        if (motiontime >= 10 && motiontime < 300)
        {
            rate_count1++;

            double rate = rate_count1 / 200.0; // needs count to 200
            Kp[0] = 30.0; Kp[1] = 30.0; Kp[2] = 30.0;
            Kd[0] = 3.0; Kd[1] = 3.0; Kd[2] = 3.0;

            qDes[0] = jointLinearInterpolation(qInit[0], qLegsdesRef[0], rate);
            qDes[1] = jointLinearInterpolation(qInit[1], qLegsdesRef[1], rate);
            qDes[2] = jointLinearInterpolation(qInit[2], qLegsdesRef[2], rate);
        }

        if (motiontime >= 300 && motiontime%1400 < 750)
        {
            rate_count2++;

            double rate = rate_count2 / 500.0; // needs count to 200

            qDes[0] = jointLinearInterpolation(qLegsdesRef[0], qLegsdes[0], rate);
            qDes[1] = jointLinearInterpolation(qLegsdesRef[1], qLegsdes[1], rate);
            qDes[2] = jointLinearInterpolation(qLegsdesRef[2], qLegsdes[2], rate);
        }

        if (motiontime%1400 > 750 && motiontime%1400 < 1399)
        {
            rate_count2++;

            double rate = rate_count2 / 500.0; // needs count to 200

            qDes[0] = jointLinearInterpolation(qLegsdes[0], qLegsdesRef[0], rate);
            qDes[1] = jointLinearInterpolation(qLegsdes[1], qLegsdesRef[1], rate);
            qDes[2] = jointLinearInterpolation(qLegsdes[2], qLegsdesRef[2], rate);
        }
        if(motiontime%1400 == 0)
        {
            rate_count1 = 0;
            rate_count2 = 0;
        }

        for (int i = 0; i < 3; i++)
        {
            low_cmd.motor_cmd()[i].q() = qDes[i];
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = 50;
            low_cmd.motor_cmd()[i].kd() = 3;
            low_cmd.motor_cmd()[i].tau() = 0;
        }
    }

    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    
    lowcmd_publisher->Write(low_cmd);
    
    
    
    //  for(int i(0); i < 4; ++i){
    //     qLegs.col(i)(0) = low_state.motor_state()[3*i].q();
    //     qLegs.col(i)(1) = low_state.motor_state()[3*i +1].q();
    //     qLegs.col(i)(2) = low_state.motor_state()[3*i +2].q();
    //  }
    // Vec34 vecXP;
    // for(int i(0); i < 4; ++i){
    //     vecXP.col(i) = _Legs[i]->calcPEe2H(qLegs.col(i));
    // }

    // for(int i(0); i < 4; ++i){
    //     cout << vecXP.col(i)(0) << " "<< vecXP.col(i)(1) << " "<< vecXP.col(i)(2) <<endl;
    //     }
    
}

int main(int argc, const char** argv)
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
