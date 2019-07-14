#define PI_RAD  0.0174532925199 // 角度转换为弧度参数
#define PI_DEG 57.2957795130823 // 弧度转换为角度参数
#include "./../kinematics/Kine.h"
#include <iostream>


int main(int argc, char **argv)
{
    double Robot_Link_Len[6] = {0.1633,0.0934,0.2932,0.2568,0.1466,0.0934}; //robot link length
    double descartes_point[6];
    Kine_IR_FiveDoF Arm5d;
    Arm5d.Set_Length(Robot_Link_Len);
    double current_joint_value[5] = {0,90,90,0,0};  //unit:degree
    double new_joint_value[5]; //rad
    double new_decartes_point[6] = {0.1,0.1,0.9,180,0,0}; //new cartesian point (xyzwpr) unit:(meter,degree)
    double toppoint[6];
    Arm5d.FKine(current_joint_value,toppoint);
    for (int i=0;i<6;i++){
        std::cout<<toppoint[i]<<std::endl;
    }
    std::cout<<"－－－－－－－－－－－－－－－－－"<<std::endl;
    


    Arm5d.IKine(new_decartes_point,current_joint_value,new_joint_value);
    if (new_decartes_point[0] == 0 && new_decartes_point[1] == 0)
        new_joint_value[0] += 89.9069;
    for (int i=0;i<5;i++){
        std::cout<<new_joint_value[i]<<std::endl;
    }
    std::cout<<"－－－－－－－－－－－－－－－－－"<<std::endl;

    return 0;
}