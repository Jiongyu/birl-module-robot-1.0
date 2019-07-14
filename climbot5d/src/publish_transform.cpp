#include "include/change_base_link.h"
#include <iostream>

bool open_gripper = false;
sensor_msgs::JointState joint_state;

void pose_callback(const sensor_msgs::JointState& msg){
    joint_state = msg;
    //if the value of G0 is 0.0, the G0_gripper will close.
    if (msg.position[0] == 0.0 ){
        std::cout<<"G0_gripper closed!!!"<<std::endl;
        open_gripper = false;
    }
    //if the value of G6 is 0.0, the G6_gripper_B will close.
    else if(msg.position[6] == 0.0){
        std::cout<<"G6_gripper closed!!!"<<std::endl;
        open_gripper = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_transform");
    ros::NodeHandle nh;
    geometry_msgs::TransformStamped tran;
    Change_base_link Climbot5d;

    std::vector<std::string> joint_names;
    joint_names = {"G0_Joint", "I1_Joint", "T2_Joint", "T3_Joint", "T4_Joint", "I5_Joint", "G6_Joint"};
    joint_state.name = joint_names;
    joint_state.position.resize(joint_names.size());
    joint_state.velocity.resize(joint_names.size());
    
    ros::Publisher pub = nh.advertise<geometry_msgs::TransformStamped>("/climbot5d/transform",20);
    ros::Subscriber sub_pose = nh.subscribe("/climbot5d/joint_state", 20, pose_callback); 

    tf::StampedTransform trans_B_to_A;
    tf::StampedTransform trans_B_to_world;
    tf::StampedTransform trans_A_to_world;

    // listen and save transform : G6_gripper to pripper G0_gripper 
    Climbot5d.save_B_to_A_transformation(trans_B_to_A);
    // listen and save transform : G6_gripper to  world 
    Climbot5d.save_B_to_world_transformation(trans_B_to_world);
    // caculate the transform : G0_gripper to world
    Climbot5d.caculate_A_to_world_transform(trans_B_to_world,trans_B_to_A,trans_A_to_world);
    //exchange trans_A_to_world to message
    tf::transformStampedTFToMsg(trans_A_to_world,tran);

    ros::Rate timer(1);
    while(ros::ok()){
        if (!open_gripper){
            while(ros::ok() && !open_gripper){
                pub.publish(tran);
                ros::spinOnce();
                timer.sleep();
            } 
        }
        if(open_gripper){ 
            Climbot5d.save_B_to_world_transformation(trans_B_to_world);
            while(ros::ok()&&open_gripper){
                ros::Time t1= ros::Time::now(); 
                Climbot5d.save_B_to_A_transformation(trans_B_to_A);
                Climbot5d.caculate_A_to_world_transform(trans_B_to_world,trans_B_to_A,trans_A_to_world);
                tf::transformStampedTFToMsg(trans_A_to_world,tran);
                pub.publish(tran);
                ROS_INFO("caculating time: %f s.\n",(ros::Time::now()-t1).toSec());
                ros::spinOnce();
                timer.sleep();
            }
        }
    }
    return 0;
}