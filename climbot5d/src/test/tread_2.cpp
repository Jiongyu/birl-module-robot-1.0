//ros
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Transform.h>
//#include <iostream>

int main(int argc, char **argv)
{   
    
    // listen and save transform : G6_gripper to pripper G0_gripper
    void save_B_to_A_transformation(tf::StampedTransform &transform);
    // listen and save transform : G6_gripper to  world 
    void save_B_to_world_transformation(tf::StampedTransform &transform);
    // caculate the transform : G0_gripper to world
    void caculate_A_to_world_transform(tf::StampedTransform& transform_B_to_world,\
                                                     tf::StampedTransform& transform_B_to_A,\
                                                     tf::Transform& transform_A_to_world);
    
    ros::init(argc, argv, "change_base_link");
    ros::NodeHandle nh;
    ros::Publisher pub_joint = nh.advertise<sensor_msgs::JointState>("/climbot5d/joint_state", 1);
    ros::Publisher pub_tran = nh.advertise<geometry_msgs::TransformStamped>("/climbot5d/transform",100);
    geometry_msgs::TransformStamped tran;
    sensor_msgs::JointState joint_state;

    while(!nh.hasParam("/joint_names")){
    ROS_INFO("Waiting for parameter '/joint_names'");
    sleep(1);
    }

    std::vector<std::string> joint_names;
    nh.getParam("/joint_names",joint_names);
    joint_state.name = joint_names;
    joint_state.position.resize(joint_names.size());
    joint_state.velocity.resize(joint_names.size());

    tf::StampedTransform trans_B_to_A;
    tf::StampedTransform trans_B_to_world;
    tf::StampedTransform trans_A_to_world;

   
    save_B_to_A_transformation(trans_B_to_A);
    save_B_to_world_transformation(trans_B_to_world);
    caculate_A_to_world_transform(trans_B_to_world,trans_B_to_A,trans_A_to_world);
    //exchange trans_A_to_world to message
    tf::transformStampedTFToMsg(trans_A_to_world,tran);

    //ros::Duration(20).sleep();
    double y = 0.0; 

    ros::Rate timer(30);

        
    while(ros::ok()&&y < 1.57){
        y += 0.01;
        joint_state.position.push_back(0.0);
        joint_state.position.push_back(0.0);
        joint_state.position.push_back(-y);
        joint_state.position.push_back(0.0);
        joint_state.position.push_back(y);
        joint_state.position.push_back(0.0);
        joint_state.position.push_back(0.04);
        pub_joint.publish(joint_state);
        pub_tran.publish(tran);
        joint_state.position.clear();
        ros::spinOnce();
        timer.sleep();
    }
    //ros::Duration(20).sleep();
        
    while(ros::ok()){
        // listen and save transform : G6_gripper to  world 
        save_B_to_world_transformation(trans_B_to_world);
        while (ros::ok()&& y >1.3){
            y -= 0.01;   
            joint_state.position.push_back(0.04);
            joint_state.position.push_back(0.0);
            joint_state.position.push_back(-y);
            joint_state.position.push_back(0);
            joint_state.position.push_back(y);
            joint_state.position.push_back(0.0);
            joint_state.position.push_back(0.0);
            pub_joint.publish(joint_state);
            save_B_to_A_transformation(trans_B_to_A);
            caculate_A_to_world_transform(trans_B_to_world,trans_B_to_A,trans_A_to_world);
            tf::transformStampedTFToMsg(trans_A_to_world,tran);
            pub_tran.publish(tran);
            joint_state.position.clear();
            ros::spinOnce();
            timer.sleep();
        }
        ros::Duration(3).sleep();
        double I5 = 0.0;
        while(ros::ok()&&I5<=3.14)
        {
            I5 += 0.01;
            joint_state.position.push_back(0.04);
            joint_state.position.push_back(0.0);
            joint_state.position.push_back(-y);
            joint_state.position.push_back(0);
            joint_state.position.push_back(y);
            joint_state.position.push_back(I5);
            joint_state.position.push_back(0.0);
            pub_joint.publish(joint_state);
            save_B_to_A_transformation(trans_B_to_A);
            caculate_A_to_world_transform(trans_B_to_world,trans_B_to_A,trans_A_to_world);
            tf::transformStampedTFToMsg(trans_A_to_world,tran);
            pub_tran.publish(tran);
            joint_state.position.clear();
            ros::spinOnce();
            timer.sleep();
        }
        ros::Duration(3).sleep();
        while(ros::ok()&&y<=1.57){
            y+=0.01;
            joint_state.position.push_back(0.04);
            joint_state.position.push_back(0.0);
            joint_state.position.push_back(-y);
            joint_state.position.push_back(0);
            joint_state.position.push_back(y);
            joint_state.position.push_back(I5);
            joint_state.position.push_back(0.0);
            pub_joint.publish(joint_state);
            save_B_to_A_transformation(trans_B_to_A);
            caculate_A_to_world_transform(trans_B_to_world,trans_B_to_A,trans_A_to_world);
            tf::transformStampedTFToMsg(trans_A_to_world,tran);
            pub_tran.publish(tran);
            joint_state.position.clear();
            ros::spinOnce();
            timer.sleep();
        }
        ros::Duration(3).sleep();
        while(ros::ok()&& y>1.3){
            y -= 0.001;   
            joint_state.position.push_back(0);
            joint_state.position.push_back(0.0);
            joint_state.position.push_back(-y);
            joint_state.position.push_back(0);
            joint_state.position.push_back(y);
            joint_state.position.push_back(I5);
            joint_state.position.push_back(0.04);
            pub_joint.publish(joint_state);
            joint_state.position.clear();
            ros::spinOnce();
            timer.sleep();
        }
        ros::Duration(3).sleep();
        double I1 = 0.0;
        while(ros::ok()&&I1<=3.14){
            I1 += 0.001;
            joint_state.position.push_back(0.0);
            joint_state.position.push_back(I1);
            joint_state.position.push_back(-y);
            joint_state.position.push_back(0);
            joint_state.position.push_back(y);
            joint_state.position.push_back(I5);
            joint_state.position.push_back(0.04);
            pub_joint.publish(joint_state);
            joint_state.position.clear();
            ros::spinOnce();
            timer.sleep();
        }
        ros::Duration(3).sleep();
        while(ros::ok()&& y<=1.57){
            y += 0.001;   
            joint_state.position.push_back(0);
            joint_state.position.push_back(I1);
            joint_state.position.push_back(-y);
            joint_state.position.push_back(0);
            joint_state.position.push_back(y);
            joint_state.position.push_back(I5);
            joint_state.position.push_back(0.04);
            pub_joint.publish(joint_state);
            joint_state.position.clear();
            ros::spinOnce();
            timer.sleep();
    }
    }
    return 0;
}

void save_B_to_A_transformation(tf::StampedTransform &transform){

    tf::TransformListener listener;
    try{            //exchange trans_A_to_world to message
        listener.waitForTransform( "/base_gripper1","/G6_tcp", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform( "/base_gripper1","/G6_tcp",ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void save_B_to_world_transformation(tf::StampedTransform &transform){

    tf::TransformListener listener;
    try{
        listener.waitForTransform( "/world","/G6_tcp", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform( "/world","/G6_tcp",ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void caculate_A_to_world_transform(tf::StampedTransform& transform_B_to_world,\
                                                     tf::StampedTransform& transform_B_to_A,\
                                                     tf::Transform& transform_A_to_world){
    transform_A_to_world  = transform_B_to_world * transform_B_to_A.inverse();
}