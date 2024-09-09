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

#include <unitree/robot/go2/robot_state/robot_state_client.hpp>
#include "common/unitreeLeg.h"
#include "common/mathTools.h"
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
    void activateService(const std::string& serviceName,int activate);
    void InitRobotStateClient();
    int queryServiceStatus(const std::string& serviceName);

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void *messages);
    void LowCmdWrite();
 
    Vec12 Freestandcmd(float row, float pitch, float yaw, float height);
    

private:

    double stand_up_joint_pos[12] = {0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
                                     0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
    double stand_down_joint_pos[12] = {0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
                                       1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};
    double dt = 0.002;
    double runing_time = 0.0;
    double phase = 0.0;

    unitree::robot::go2::RobotStateClient rsc;

    QuadrupedLeg* _Legs[4];

    float Pz = 0.37;
    int freemode = 0;
    int set_freemode = 0;

    Vec12 freeQ;

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

    std::cout << "X(ROLL):0;Y(YAW):1;Z(PITCH):2;Height:3;MIx:4"<< endl;
    std::cin >> freemode; 
    std::cout << "freemode:" << freemode << endl;
}

Vec12 Custom::Freestandcmd(float row, float pitch, float yaw, float height){
    Vec34 _initVecXP, qLegs;
    Vec3 _initVecOX;
    for(int i(0); i < 4; ++i){
         qLegs.col(i)(0) = low_state.motor_state()[3*i].q();
         qLegs.col(i)(1) = low_state.motor_state()[3*i + 1].q();
         qLegs.col(i)(2) = low_state.motor_state()[3*i + 2].q();
     }
    Vec3 x = _Legs[0]->calcPEe2B(qLegs.col(0));
    _initVecOX = _Legs[0]->calcPEe2B(qLegs.col(0));
    
    for(int i(0); i < 4; ++i){
        _initVecXP.col(i) = _Legs[i]->calcPEe2B(qLegs.col(i)) - x;
    }

    Vec34 vecPsi; 
    Vec12 qDes;
    FrameType BODY;

    Vec3 vecXO = -_initVecOX;
    vecXO(2) += height;

    RotMat rotM = rpyToRotMat(row, pitch, yaw);

    HomoMat Tsb = homoMatrix(vecXO, rotM);
    HomoMat Tbs = homoMatrixInverse(Tsb);

    Vec4 tempVec4;
    Vec34 vecOP;
    for(int i(0); i<4; ++i){
        tempVec4 = Tbs * homoVec(_initVecXP.col(i));
        vecOP.col(i) = noHomoVec(tempVec4);
    }
    vecPsi = vecOP;

    Vec12 q;
    for(int i(0); i < 4; ++i){
        q.segment(3*i, 3) = _Legs[i]->calcQ(vecPsi.col(i), BODY);
    }
    qDes = q;

    return qDes;
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
            low_cmd.motor_cmd()[i].kd() = 8;
            low_cmd.motor_cmd()[i].tau() = 0;
        }
    }
    else if(runing_time >= 3.0 && runing_time < 4.0)
    {
        // Then stand
        phase = tanh((runing_time - 3.0) / 1.2);
        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd()[i].q() = phase * stand_up_joint_pos[i] + (1 - phase) * stand_up_joint_pos[i];
            low_cmd.motor_cmd()[i].dq() = 0;
            if(i%3 == 0){
                low_cmd.motor_cmd()[i].kp() = 55;
            }
            else if(i%3 == 1){
                low_cmd.motor_cmd()[i].kp() = 55;
            }
            else if(i%3 == 2){
                low_cmd.motor_cmd()[i].kp() = 85;
            }
            low_cmd.motor_cmd()[i].kd() = 5;
            if(i%3 == 2){
                low_cmd.motor_cmd()[i].kd() = 7;
            }
            low_cmd.motor_cmd()[i].tau() = 0;
        }
        if(set_freemode == 0){
            set_freemode = 1;
            switch(freemode){
            case 0:
                freeQ = Freestandcmd(0.25, 0.0, 0, 0);  //X:14.3 0.25  Y:0 0  Z:0 0
            break;
            case 1:
                freeQ = Freestandcmd(0, 0.25, 0, 0);  //X:0 0  Y:14.3 0.25  Z:0 0
            break;
            case 2:
                freeQ = Freestandcmd(0, 0.0, 0.25, 0);  //X:0 0  Y:0 0  Z:14.3 0.25
            break;
            case 3:
                freeQ = Freestandcmd(0, 0.0, 0, 0.04);  //X:0 0  Y:0 0  Z:0 0
            break;
            case 4:
                freeQ = Freestandcmd(0.10, 0.10, 0.10, 0);  //X:5.7 0.10  Y:5.7 0.10  Z:5.7 0.10
            break;
            }
        }
    }
    else if(runing_time >= 4.0 && runing_time < 6.0)
    {
        // Then free stand
        phase = tanh((runing_time - 4.0) / 1.2);
        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd()[i].q() = phase * freeQ[i] + (1 - phase) * stand_up_joint_pos[i];
            low_cmd.motor_cmd()[i].dq() = 0;
            if(i%3 == 0){
                low_cmd.motor_cmd()[i].kp() = 55;
            }
            else if(i%3 == 1){
                low_cmd.motor_cmd()[i].kp() = 55;
            }
            else if(i%3 == 2){
                low_cmd.motor_cmd()[i].kp() = 85;
            }
            low_cmd.motor_cmd()[i].kd() = 5;
            if(i%3 == 2){
                low_cmd.motor_cmd()[i].kd() = 7;
            }
            low_cmd.motor_cmd()[i].tau() = 0;
        }
    }
    else if(runing_time >= 6.0 && runing_time < 8.0)
    {
        // Then free stand
        phase = tanh((runing_time - 6.0) / 1.2);
        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd()[i].q() = phase * stand_up_joint_pos[i] + (1 - phase) * freeQ[i];
            low_cmd.motor_cmd()[i].dq() = 0;
            if(i%3 == 0){
                low_cmd.motor_cmd()[i].kp() = 55;
            }
            else if(i%3 == 1){
                low_cmd.motor_cmd()[i].kp() = 55;
            }
            else if(i%3 == 2){
                low_cmd.motor_cmd()[i].kp() = 85;
            }
            low_cmd.motor_cmd()[i].kd() = 5;
            if(i%3 == 2){
                low_cmd.motor_cmd()[i].kd() = 7;
            }
            low_cmd.motor_cmd()[i].tau() = 0;
        }
    }
    else
    {
        // Then free stand
        phase = tanh((runing_time - 8.0) / 1.2);
        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd()[i].q() = phase * stand_down_joint_pos[i] + (1 - phase) * stand_up_joint_pos[i];
            low_cmd.motor_cmd()[i].dq() = 0;
            if(i%3 == 0){
                low_cmd.motor_cmd()[i].kp() = 55;
            }
            else if(i%3 == 1){
                low_cmd.motor_cmd()[i].kp() = 55;
            }
            else if(i%3 == 2){
                low_cmd.motor_cmd()[i].kp() = 85;
            }
            low_cmd.motor_cmd()[i].kd() = 5;
            if(i%3 == 2){
                low_cmd.motor_cmd()[i].kd() = 7;
            }
            low_cmd.motor_cmd()[i].tau() = 0;
        }
    }

    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);
}

void Custom::InitRobotStateClient()
{
    rsc.SetTimeout(10.0f); 
    rsc.Init();
}

void Custom::activateService(const std::string& serviceName,int activate)
{
    rsc.ServiceSwitch(serviceName, activate);  
}

int Custom::queryServiceStatus(const std::string& serviceName)
{
    std::vector<unitree::robot::go2::ServiceState> serviceStateList;
    int ret,serviceStatus;
    ret = rsc.ServiceList(serviceStateList);
    size_t i, count=serviceStateList.size();
    for (i=0; i<count; i++)
    {
        const unitree::robot::go2::ServiceState& serviceState = serviceStateList[i];
        if(serviceState.name == serviceName)
        {
            if(serviceState.status == 0)
            {
                std::cout << "name: " << serviceState.name <<" is activate"<<std::endl;
                serviceStatus = 1;
            }
            else
            {
                std::cout << "name:" << serviceState.name <<" is deactivate"<<std::endl;
                serviceStatus = 0;
            } 
        }    
    }
    return serviceStatus;
    
}

int main(int argc, const char **argv)
{
    int i = 0;
    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(1, "lo");
    }
    else
    {
        ChannelFactory::Instance()->Init(0, argv[1]);  //go2
        i = 1;
    }
    Custom custom;
    if(i == 1)
    {
        std::cout << "WARNING: Make sure the robot is hung up or lying on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        custom.InitRobotStateClient();
        while(custom.queryServiceStatus("sport_mode"))
        {
        std::cout<<"Try to deactivate the service: "<<"sport_mode"<<std::endl;
        custom.activateService("sport_mode",0);
        sleep(1);
        } 
    }
    std::cout << "Press enter to start";
    std::cin.get();
    custom.Init();
    while (1)
    {
        sleep(10);
    }

    return 0;
}
