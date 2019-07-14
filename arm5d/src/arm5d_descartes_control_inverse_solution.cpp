#define PI_RAD  0.0174532925199 // 角度转换为弧度参数
#define PI_DEG 57.2957795130823 // 弧度转换为角度参数
#include "./../kinematics/Kine.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

std_msgs::Float64MultiArray request_descartes_point;

int main(int argc, char **argv)
{
    void subCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
    double Robot_Link_Len[6] = {0.1633,0.0934,0.2932,0.2568,0.1466,0.0934}; //robot link length
    double descartes_point[6];
    Kine_IR_FiveDoF Arm5d;
    Arm5d.Set_Length(Robot_Link_Len);

    ros::init(argc, argv, "arm5d_descartes_control_inverse_solution");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("Arm5d_Inverse_solution", 10);
    ros::Subscriber sub = nh.subscribe("Arm5d_Descartes_point", 10, subCallback);
    while(ros::ok() && sub.getNumPublishers()<1){
        ROS_INFO("topic Descartes_point not exist!!");
        sleep(1);
    }

    std_msgs::Float64MultiArray transmit_joint_list; // I1,T2,T3,i4,t5
    transmit_joint_list.data.resize(5);
    request_descartes_point.data.resize(11);  //descartes points + joint points


    double current_joint_value[5] = {0,90,90,0,0};  //unit:degree
    double new_joint_value[5]; //rad
    double new_decartes_point[6] = {0,0,1.0467,180,0,0}; //new cartesian point (xyzwpr) unit:(meter,degree)
    ros::Rate timer(50);
    while (ros::ok())
    {
        if(abs(new_decartes_point[0] - request_descartes_point.data[0]) < 1e-5 ||
            abs(new_decartes_point[1] - request_descartes_point.data[1]) < 1e-5 ||
            abs(new_decartes_point[2] - request_descartes_point.data[2]) < 1e-5 ||
            abs(new_decartes_point[3] - request_descartes_point.data[3]*PI_DEG) < 1e-5 ||
            abs(new_decartes_point[4] - request_descartes_point.data[4]*PI_DEG) < 1e-5 ||
            abs(new_decartes_point[5] - request_descartes_point.data[5]*PI_DEG) < 1e-5 )
        {
            new_decartes_point[0] = request_descartes_point.data[0];  //X   
            new_decartes_point[1] = request_descartes_point.data[1];  //Y   
            new_decartes_point[2] = request_descartes_point.data[2];  //Z
            new_decartes_point[3] = request_descartes_point.data[3]*PI_DEG;  //RX
            new_decartes_point[4] = request_descartes_point.data[4]*PI_DEG;  //RY   
            new_decartes_point[5] = (request_descartes_point.data[5])*PI_DEG;  //RZ

            current_joint_value[0] = request_descartes_point.data[6]*PI_DEG; //I1
            current_joint_value[1] = 90 + request_descartes_point.data[7]*PI_DEG; //T2
            current_joint_value[2] = 90 + request_descartes_point.data[8]*PI_DEG; //T3
            current_joint_value[3] = request_descartes_point.data[9]*PI_DEG; //i4
            current_joint_value[4] = request_descartes_point.data[10]*PI_DEG; //t5

            Arm5d.IKine(new_decartes_point,current_joint_value,new_joint_value);
            
            if (new_decartes_point[0] == 0 && new_decartes_point[1] == 0)
            new_joint_value[0] += 89.9069;
            transmit_joint_list.data[0] = new_joint_value[0]*PI_RAD; //I1
            transmit_joint_list.data[1] = new_joint_value[1]*PI_RAD - 1.570796 ; //T2
            transmit_joint_list.data[2] = new_joint_value[2]*PI_RAD - 1.570796 ; //T3
            transmit_joint_list.data[3] = new_joint_value[3]*PI_RAD; //i4
            transmit_joint_list.data[4] = new_joint_value[4]*PI_RAD; //t5

            pub.publish(transmit_joint_list);
        }
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}

void subCallback(const std_msgs::Float64MultiArrayConstPtr &msg){
    request_descartes_point = *msg;
}

