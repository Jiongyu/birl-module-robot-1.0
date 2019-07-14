#include "./../kinematics/Kine.h"
#include "ros/ros.h"
#include "std_msgs/Int64MultiArray.h"

int main(int argc, char **argv)
{
    void Current_joint_Rad_to_Deg(double (&joint_value)[5]);
    double New_joint_value[5];//unit:degree
    double Robot_Link_Len[6] = {0.1764,0.2568,0.2932,0.2932,0.2568,0.1764}; //robot link length
    Kine_CR_FiveDoF_G1 Climbot_G1; // robot based on the gripper1 to get inverse solution
    Kine_CR_FiveDoF_G2 Climbot_G2; // robot based on the gripper6 to get inverse solution
    Climbot_G1.Set_Length(Robot_Link_Len);
    Climbot_G2.Set_Length(Robot_Link_Len);
    double Current_joint_value[5] = {0,0,0,0,0};  //unit:degree
    double New_point[6]; //new cartesian point (xyzwpr(RxRyRz)) unit:(meter,degree)
    ///end
    
    double Top_point[6];
    Climbot_G1.FKine(Current_joint_value,Top_point);
    for(int i = 0; i < 6; i++){
        std::cout<<Top_point[i]<<std::endl;
    }
    std::cout<<"-------------------------------------"<<std::endl;
    Climbot_G2.FKine(Current_joint_value,Top_point);
    for(int i = 0; i < 6; i++){
        std::cout<<Top_point[i]<<std::endl;
    }

    std::cout<<"-----------inversion position--------------------------"<<std::endl;
    New_point[0] = 0.57;
    New_point[1] = 0;
    New_point[2] = 0;
    New_point[3] = 0;
    New_point[4] = 0;
    New_point[5] = 180;
    // Current_joint_Rad_to_Deg(Current_joint_value);
    Climbot_G1.IKine(New_point,Current_joint_value,New_joint_value);

    for(int i = 0; i < 5; i++){
        std::cout<<New_joint_value[i]<<std::endl;
    }

    
    double current_top_velocity[6] = {-0.06,0,0,0,0,0}; // m/s,degree/s
    double new_joint_velocity[5] = {0,0,0,0,0}; // degree/s

    std::cout<<"velocity_inverse-----------------------------"<<std::endl;
    Climbot_G1.Vel_IKine(New_joint_value,current_top_velocity,new_joint_velocity);
    for(int i = 0; i < 5; i++){
        std::cout<<new_joint_velocity[i]<<std::endl;
    }

    std::cout<<"velocity_positive-----------------------------"<<std::endl;
    new_joint_velocity[0] = 0;
    new_joint_velocity[1] = 8.32102;
    new_joint_velocity[2] = -16.642;
    new_joint_velocity[3] = 8.32102;
    new_joint_velocity[4] = 0;

    Climbot_G1.Vel_FKine(New_joint_value,new_joint_velocity,current_top_velocity);
    for(int i = 0; i < 6; i++){
        std::cout<<current_top_velocity[i]<<std::endl;
    }
    return 0;
}

void Current_joint_Rad_to_Deg(double (&joint_value)[5]){
    joint_value[0]*=PI_DEG;
    joint_value[1]*=PI_DEG;
    joint_value[2]*=PI_DEG;
    joint_value[3]*=PI_DEG;
    joint_value[4]*=PI_DEG;
}