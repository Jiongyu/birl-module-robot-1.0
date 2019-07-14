#define PI_RAD  0.0174532925199 // 角度转换为弧度参数
#define PI_DEG 57.2957795130823 // 弧度转换为角度参数
#include "./../kinematics/Kine.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

std_msgs::Float64MultiArray request_joint_point;

int main(int argc, char **argv)
{
    void subCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
    double Robot_Link_Len[6] = {0.1633,0.0934,0.2932,0.2568,0.1466,0.0934}; //robot link length
    double actual_joint_value[5] = {0,90,90,0,0};
    double descartes_point[6];
    Kine_IR_FiveDoF Arm5d;
    Arm5d.Set_Length(Robot_Link_Len);

    ros::init(argc, argv, "arm5d_joint_control_positive_solution");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("Arm5d_Positive_solution", 10);
    ros::Subscriber sub = nh.subscribe("Arm5d_Joint_point", 10, subCallback);
    while(ros::ok() && sub.getNumPublishers() < 1){
        ROS_INFO("Topic Joint_point not exist!!");
        sleep(1);
    }

    std_msgs::Float64MultiArray transmit_descartes_point;
    transmit_descartes_point.data.resize(6);
    request_joint_point.data.resize(6);
    ros::Rate timer(50);
    while(ros::ok()){

        if ((abs(actual_joint_value[0] - request_joint_point.data[1]*PI_DEG))>1e-3 ||
            (abs(actual_joint_value[1] - request_joint_point.data[2]*PI_DEG))>1e-3 ||
            (abs(actual_joint_value[2] - request_joint_point.data[3]*PI_DEG))>1e-3 ||
            (abs(actual_joint_value[3] - request_joint_point.data[4]*PI_DEG))>1e-3 ||
            (abs(actual_joint_value[4] - request_joint_point.data[5]*PI_DEG))>1e-3){
                
                actual_joint_value[0] = request_joint_point.data[1]*PI_DEG;
                actual_joint_value[1] = 90 + request_joint_point.data[2]*PI_DEG;
                actual_joint_value[2] = 90 + request_joint_point.data[3]*PI_DEG;
                actual_joint_value[3] = request_joint_point.data[4]*PI_DEG;
                actual_joint_value[4] = request_joint_point.data[5]*PI_DEG;
                
                Arm5d.FKine(actual_joint_value,descartes_point);

                transmit_descartes_point.data[0] = descartes_point[0];
                transmit_descartes_point.data[1] = descartes_point[1];
                transmit_descartes_point.data[2] = descartes_point[2];
                transmit_descartes_point.data[3] = descartes_point[3]*PI_RAD;
                transmit_descartes_point.data[4] = descartes_point[4]*PI_RAD;
                transmit_descartes_point.data[5] = descartes_point[5]*PI_RAD;
                pub.publish(transmit_descartes_point);
            }

        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}

void subCallback(const std_msgs::Float64MultiArrayConstPtr &msg){
    request_joint_point = *msg;
}
