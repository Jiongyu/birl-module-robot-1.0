#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Transform.h>

sensor_msgs::JointState receive_joint_command;

int main(int argc, char **argv)
{   
    void command_Callback(const sensor_msgs::JointStateConstPtr &msg);
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
    ros::Publisher pub_joint_command = nh.advertise<sensor_msgs::JointState>("/climbot5d/simulation_pub_joint_command", 20);
    ros::Publisher pub_tran = nh.advertise<geometry_msgs::TransformStamped>("/climbot5d/transform",20);
    ros::Subscriber sub_joint_command = nh.subscribe("/climbot5d_simulation_joint_command", 20, command_Callback);

    /*
    while(sub_joint_command.getNumPublishers()<1){
        std::cout<<"topic: /climbot5d_simulation_joint_command is not exist!!!!"<<std::endl;
        sleep(1); 
    }*/
   
    geometry_msgs::TransformStamped tran;
    sensor_msgs::JointState msg_joint_command;

    msg_joint_command.name.resize(7);
    std::vector<std::string> joint_names;
    joint_names = {"G0_Joint", "I1_Joint", "T2_Joint", "T3_Joint", "T4_Joint", "I5_Joint", "G6_Joint"};
    msg_joint_command.name = joint_names;
    msg_joint_command.position.resize(joint_names.size());
    receive_joint_command.name = joint_names;
    receive_joint_command.position.resize(joint_names.size());

    tf::StampedTransform trans_B_to_A;
    tf::StampedTransform trans_B_to_world;
    tf::StampedTransform trans_A_to_world;

    save_B_to_A_transformation(trans_B_to_A);
    save_B_to_world_transformation(trans_B_to_world);
    caculate_A_to_world_transform(trans_B_to_world,trans_B_to_A,trans_A_to_world);
    //exchange trans_A_to_world to message
    tf::transformStampedTFToMsg(trans_A_to_world,tran);

    msg_joint_command.position[0] = 0;
    msg_joint_command.position[1] = 0;
    msg_joint_command.position[2] = -1.57;
    msg_joint_command.position[3] = 0;
    msg_joint_command.position[4] = 1.57;
    msg_joint_command.position[5] = 0;
    msg_joint_command.position[6] = 0;
    pub_joint_command.publish(msg_joint_command);
    ros::spinOnce();
    double G0_value,I1_value,T2_value,T3_value,T4_value,I5_value,G6_value;
    ros::Rate timer(40);
    while(ros::ok()){
        if (G0_value != receive_joint_command.position[0] ||
            I1_value != receive_joint_command.position[1] ||
            T2_value != receive_joint_command.position[2] ||
            T3_value != receive_joint_command.position[3] ||
            T4_value != receive_joint_command.position[4] ||
            I5_value != receive_joint_command.position[5] ||
            G6_value != receive_joint_command.position[6]){

                G0_value = receive_joint_command.position[0];
                I1_value = receive_joint_command.position[1];
                T2_value = receive_joint_command.position[2];
                T3_value = receive_joint_command.position[3];
                T4_value = receive_joint_command.position[4];
                I5_value = receive_joint_command.position[5];
                G6_value = receive_joint_command.position[6];

                if (receive_joint_command.position[6] == 600){
                    msg_joint_command.position[0] = 0;
                    msg_joint_command.position[1] = receive_joint_command.position[1];
                    msg_joint_command.position[2] = -1.57 + receive_joint_command.position[2];
                    msg_joint_command.position[3] = -receive_joint_command.position[3];
                    msg_joint_command.position[4] = 1.57 - receive_joint_command.position[4];
                    msg_joint_command.position[5] = receive_joint_command.position[5];
                    msg_joint_command.position[6] = 0.04;

                    pub_joint_command.publish(msg_joint_command);
                }

                 // listen and save transform : G6_gripper to  world 
                save_B_to_world_transformation(trans_B_to_world);

                if(receive_joint_command.position[0] == 600){
                    msg_joint_command.position[0] = 0.04;
                    msg_joint_command.position[1] = receive_joint_command.position[1];
                    msg_joint_command.position[2] = -1.57 + receive_joint_command.position[2];
                    msg_joint_command.position[3] = - receive_joint_command.position[3];
                    msg_joint_command.position[4] =  1.57 - receive_joint_command.position[4];
                    msg_joint_command.position[5] = receive_joint_command.position[5];
                    msg_joint_command.position[6] = 0;
                    pub_joint_command.publish(msg_joint_command);
                    save_B_to_A_transformation(trans_B_to_A);
                    caculate_A_to_world_transform(trans_B_to_world,trans_B_to_A,trans_A_to_world);
                    tf::transformStampedTFToMsg(trans_A_to_world,tran);
                    pub_tran.publish(tran);
                }
            }
            ros::spinOnce();
            timer.sleep();
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

void command_Callback(const sensor_msgs::JointStateConstPtr &msg){
    receive_joint_command = *msg;
}