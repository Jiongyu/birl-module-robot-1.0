#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_publish_joint");
    ros::NodeHandle nh;
    ros::Publisher pub_joint_command = nh.advertise<std_msgs::Float64MultiArray>("/climbot5d/joint_position_controller/command", 1);
    ros::Publisher pub_gripper_G0_command = nh.advertise<std_msgs::Float64>("/climbot5d/jointG0_effort_controller/command", 1);
    ros::Publisher pub_gripper_G01_command = nh.advertise<std_msgs::Float64>("/climbot5d/jointG01_effort_controller/command", 1);
    ros::Publisher pub_gripper_G6_command = nh.advertise<std_msgs::Float64>("/climbot5d/jointG6_effort_controller/command", 1);
    ros::Publisher pub_gripper_G61_command = nh.advertise<std_msgs::Float64>("/climbot5d/jointG61_effort_controller/command", 1);
    
    std_msgs::Float64MultiArray msg_joint;
    msg_joint.data.resize(5);
    msg_joint.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_joint.layout.dim[0].size = 5;
    msg_joint.layout.dim[0].stride = 1;
    msg_joint.layout.dim[0].label = "joint_command: I1_Joint,T2_Joint,T3_Joint,T4_Joint,I5_Joint";

    std_msgs::Float64 G0,G01,G6,G61;
    G0.data = -100;
    G01.data = -100;
    G6.data = 100;
    G61.data = 100;

/*
    std_msgs::Float64MultiArray msg_gripper;
    msg_gripper.data.resize(4);
    msg_gripper.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_gripper.layout.dim[0].size = 4;
    msg_gripper.layout.dim[0].stride = 1;
    msg_gripper.layout.dim[0].label = "gripper_command: G0_Joint,G0_Joint1,G6_Joint,G6_Joint1";
*/

    msg_joint.data[0] = 0;
    msg_joint.data[1] = -1.57;
    msg_joint.data[2] = 0;
    msg_joint.data[3] = 1.57;
    msg_joint.data[4] = 0;
/*
    msg_gripper.data[0] = -10;
    msg_gripper.data[1] = -10;
    msg_gripper.data[2] = 10;
    msg_gripper.data[3] = 10;
*/
    pub_gripper_G0_command.publish(G0);
    pub_gripper_G01_command.publish(G01);
    pub_gripper_G6_command.publish(G6);
    pub_gripper_G61_command.publish(G61);
    ros::Duration(3).sleep();
    pub_joint_command.publish(msg_joint);
    ros::spin();
    return 0;
}