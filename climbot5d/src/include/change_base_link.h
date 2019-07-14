//ros
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Transform.h>

#include <iostream>

class Change_base_link
{
    public:   
        Change_base_link();
        Change_base_link(ros::NodeHandle* nodeHandle);

        sensor_msgs::JointState joint_state;
        geometry_msgs::TransformStamped transform;
        
        // listen and save transform : G6_gripper to pripper G0_gripper 
        void save_B_to_A_transformation(tf::StampedTransform& transform);
        // listen and save transform : G6_gripper to  world
        void save_B_to_world_transformation(tf::StampedTransform& transform);
        // caculate the transform : G0_gripper to world
        void caculate_A_to_world_transform(tf::StampedTransform& transform_B_to_world,\
                                            tf::StampedTransform& transform_B_to_A,\
                                            tf::Transform& transform_A_to_world);

        void Initial_Pub_Joint_Command(const std_msgs::String& publisher_topic);
        void Initial_Sub_Joint_Command(const std_msgs::String& subscriber_topic);
        void Initial_Pub_Joint_State(const std_msgs::String& publisher_topic);

        ~Change_base_link();
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_joint_command;
        ros::Publisher pub_joint_state;
        ros::Subscriber sub_joint_command;
    
    private:
        void Joint_Command_Callback(const sensor_msgs::JointState& msg);
        void Transform_Callback(const geometry_msgs::TransformStamped& msg);


};

Change_base_link::Change_base_link(){
    ROS_INFO("in class constructor of Change_base_link");

}

Change_base_link::Change_base_link(ros::NodeHandle* nodeHandle):nh_(*nodeHandle){

    ROS_INFO("in class constructor of Change_base_link");
    while(!nodeHandle->hasParam("/joint_names")){
    ROS_INFO("Waiting for parameter '/joint_names'");
    sleep(1);
    }
}

void Change_base_link::save_B_to_A_transformation(tf::StampedTransform &transform){

    tf::TransformListener listener;
    try{
        listener.waitForTransform( "/base_gripper1","/G6_tcp", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform( "/base_gripper1","/G6_tcp",ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void Change_base_link::save_B_to_world_transformation(tf::StampedTransform &transform){

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

void Change_base_link::caculate_A_to_world_transform(tf::StampedTransform& transform_B_to_world,\
                                                     tf::StampedTransform& transform_B_to_A,\
                                                     tf::Transform& transform_A_to_world){
    transform_A_to_world  = transform_B_to_world * transform_B_to_A.inverse();
}

void Change_base_link::Initial_Pub_Joint_Command(const std_msgs::String& publisher_topic){
    ROS_INFO("Initializing Publishers joint_command");
    pub_joint_command = nh_.advertise<sensor_msgs::JointState>(publisher_topic.data.c_str(), 1, true); 
}

void Change_base_link::Initial_Pub_Joint_State(const std_msgs::String& publisher_topic){
    ROS_INFO("Initializing Publishers joint_state");
    pub_joint_state = nh_.advertise<sensor_msgs::JointState>(publisher_topic.data.c_str(), 1, true); 
}

void Change_base_link::Initial_Sub_Joint_Command(const std_msgs::String& subscriber_topic){
    ROS_INFO("Initializing Subscribers");
    sub_joint_command = nh_.subscribe(subscriber_topic.data.c_str(), 1, &Change_base_link::Joint_Command_Callback,this);  
}

void Change_base_link::Joint_Command_Callback(const sensor_msgs::JointState& msg){
    joint_state = msg;
}

void Change_base_link::Transform_Callback(const geometry_msgs::TransformStamped& msg){
    transform = msg;
}


Change_base_link::~Change_base_link(){
    ROS_INFO("Change_base_link is nomally shutting ! ");
}