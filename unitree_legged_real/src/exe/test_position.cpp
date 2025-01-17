#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

int main(int argc, char** argv){
    // ROS node 초기화하고, example_position_without_lcm라는 이름으로 노드 생성
    ros::init(argc, argv, "example_postition_without_lcm"); 
    std::cout << "go1 컨트롤 TEST 수행"
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;

    std::cin.ignore(); // 엔터 한번 치고 실행되게 코드구성

    ros::NodeHandle nh;
    ros::Rate loop_rate(500); // 루프 속도를 초당 500Hz로 설정 

    long motiontime = 0;
    int rate_count = 0;
    int sin_count = 0;
    float qInit[3] = {0};   //초기 관절각도
    float qDes[3] = {0};    //목표 관절각도
    float sin_mid_q[3] = {0.0, 1.2, -2.0};  //중간 값
    float Kp[3] = {0};      //위치 제어 이득???
    float Kd[3] = {0};      //속도 제어 이득???

    // 로봇에 보낼 제어 명령 메시지 
    unitree_legged_msgs::LowCmd low_cmd_ros;

    bool initiated_flag = false; // initiate need time
    int count = 0;

    //pub은 low_cmd라는 ROS 토픽으로 메시지를 발행하기 위한 퍼블리셔
    ros::Publisher pub = nh.advertise<unitree_legged_msgs::LowCmd>("low_cmd", 1);


    low_cmd_ros.head[0] = 0xFE;
    low_cmd_ros.head[1] = 0xEF;
    low_cmd_ros.levelFlag = LOWLEVEL;
    
    float leg_position[4][3] = {
        {0.0, 1.2, -2.0},  // 오른쪽 앞다리 (FR_0, FR_1, FR_2)
        {0.0, 1.2, -2.0},  // 왼쪽 앞다리 (FL_0, FL_1, FL_2)
        {0.0, 1.2, -2.0},  // 오른쪽 뒷다리 (RR_0, RR_1, RR_2)
        {0.0, 1.2, -2.0}   // 왼쪽 뒷다리 (RL_0, RL_1, RL_2)
    };


    //go1 초기 설정값 
    for (int i = 0; i < 12; i++)
    {
        low_cmd_ros.motorCmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode ??
        low_cmd_ros.motorCmd[i].q = PosStopF; //이것이 뭔지 알아야겠네???
        low_cmd_ros.motorCmd[i].Kp = 5;
        low_cmd_ros.motorCmd[i].dq = VelStopF; //이것이 뭔지 알아야겠네???
        low_cmd_ros.motorCmd[i].Kd = 1;
        low_cmd_ros.motorCmd[i].tau = 0;
    }
    
    while (ros::ok())
    {
        if (initiated_flag == true){
            for (int leg = 0; leg < 4; ++leg) {
                for (int joint = 0; joint < 3; ++joint) {
                    int motor_index = leg * 3 + joint; // 모터 인덱스 계산
                    low_cmd_ros.motorCmd[motor_index].q = leg_position[leg][joint];
                    low_cmd_ros.motorCmd[motor_index].dq = 0.0; // 목표 속도 0
                }
            }
        }

        count++;
        if (count > 10)
        {
            count = 10;
            initiated_flag = true;
        }

        pub.publish(low_cmd_ros);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}