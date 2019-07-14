#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ModelState.h>
#include <sensor_msgs/JointState.h>

sensor_msgs::JointState receive_joint_command;

int main(int argc, char **argv)
{
    void callback(const sensor_msgs::JointStateConstPtr &msg);
    ros::init(argc, argv, "gazebo_publish_joint");
    ros::NodeHandle nh;
    ros::Publisher pub_joint_command = nh.advertise<std_msgs::Float64MultiArray>("/climbot5d/joint_position_controller/command", 10);
    ros::Publisher pub_gripper_G0_command = nh.advertise<std_msgs::Float64>("/climbot5d/jointG0_effort_controller/command", 10);
    ros::Publisher pub_gripper_G01_command = nh.advertise<std_msgs::Float64>("/climbot5d/jointG01_effort_controller/command", 10);
    ros::Publisher pub_gripper_G6_command = nh.advertise<std_msgs::Float64>("/climbot5d/jointG6_effort_controller/command", 10);
    ros::Publisher pub_gripper_G61_command = nh.advertise<std_msgs::Float64>("/climbot5d/jointG61_effort_controller/command", 10);
    ros::Publisher pub_model_state = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    
    std_msgs::Float64MultiArray init_joint_data;
    init_joint_data.data.resize(5);
    init_joint_data.data[0] = 0;     //I1
    init_joint_data.data[1] = -1.570796327;     //T2
    init_joint_data.data[2] = 0;     //T3
    init_joint_data.data[3] = 1.570796327;     //T4
    init_joint_data.data[4] = 0;     //I5

    std_msgs::Float64 G0,G01,G6,G61;
    //open gripper
    G0.data = 100;
    G01.data = 100;
    G6.data = 100;
    G61.data = 100;

    gazebo_msgs::ModelState robot;
    robot.model_name = "climbot5d";
    robot.pose.position.x = 0;
    robot.pose.position.y = 0 ;  
    robot.pose.position.z = 1;
    robot.pose.orientation.x = 0;
    robot.pose.orientation.y = 0;
    robot.pose.orientation.z = 0;
    robot.pose.orientation.w = 1;
    robot.twist.linear.x = 0;
    robot.twist.linear.y = 0;
    robot.twist.linear.z = 0;
    robot.twist.angular.x = 0;
    robot.twist.angular.y = 0;
    robot.twist.angular.z = 0;

    for(int i=0;i<50;i++){
        pub_gripper_G0_command.publish(G0);
        pub_gripper_G01_command.publish(G01);
        pub_gripper_G6_command.publish(G6);
        pub_gripper_G61_command.publish(G61);
        pub_joint_command.publish(init_joint_data);
        pub_model_state.publish(robot); 
        ros::Duration(0.1).sleep();
    }

    G0.data = -100;
    G01.data = -100;
    G6.data = -100;
    G61.data = -100;

    for(int i=0;i<30;i++){
        pub_gripper_G0_command.publish(G0);
        pub_gripper_G01_command.publish(G01);
        pub_gripper_G6_command.publish(G6);
        pub_gripper_G61_command.publish(G61);
        pub_joint_command.publish(init_joint_data);
        pub_model_state.publish(robot);  
        ros::Duration(0.1).sleep();
    }

    ros::Subscriber sub_command = nh.subscribe("/climbot5d_simulation_joint_command", 10, callback);
    std::vector<std::string> joint_name;
    joint_name = {"G0_Joint", "I1_Joint", "T2_Joint", "T3_Joint", "T4_Joint", "I5_Joint", "G6_Joint"};
    receive_joint_command.name.resize(joint_name.size());
    receive_joint_command.name = joint_name;
    receive_joint_command.position.resize(joint_name.size());
    receive_joint_command.position = {-100,0,0,0,0,0,-100};
    double current_joint_state[7] = {-100,0,-1.570796327,0,1.570796327,0,-100};
    std_msgs::Float64MultiArray joint_command;

    ros::Rate timer(30);
    while(ros::ok()){
            if( receive_joint_command.position[0] != current_joint_state[0] ||
                receive_joint_command.position[1] != current_joint_state[1] ||
                receive_joint_command.position[2] != current_joint_state[2] ||
                receive_joint_command.position[3] != current_joint_state[3] || 
                receive_joint_command.position[4] != current_joint_state[4] ||
                receive_joint_command.position[5] != current_joint_state[5] ||
                receive_joint_command.position[6] != current_joint_state[6] ){

                joint_command.data = {receive_joint_command.position[1],receive_joint_command.position[2],
                                    receive_joint_command.position[3],receive_joint_command.position[4],
                                    receive_joint_command.position[5]};

                G0.data = receive_joint_command.position[0];
                G01.data = receive_joint_command.position[0];
                G6.data = receive_joint_command.position[6];
                G61.data = receive_joint_command.position[6];

                pub_gripper_G0_command.publish(G0);
                pub_gripper_G01_command.publish(G01);
                pub_gripper_G6_command.publish(G6);
                pub_gripper_G61_command.publish(G61);
                pub_joint_command.publish(joint_command);

                current_joint_state[0] = receive_joint_command.position[0];
                current_joint_state[1] = receive_joint_command.position[1];
                current_joint_state[2] = receive_joint_command.position[2];
                current_joint_state[3] = receive_joint_command.position[3];
                current_joint_state[4] = receive_joint_command.position[4];
                current_joint_state[5] = receive_joint_command.position[5];
                current_joint_state[6] = receive_joint_command.position[6];
                std::cout<< "got new message!"<<std::endl;
            }
            else
                std::cout<< "no new message!"<<std::endl;
            timer.sleep();
            ros::spinOnce();

    }
    return 0;
}

void callback(const sensor_msgs::JointStateConstPtr &msg){
    receive_joint_command = *msg;
}