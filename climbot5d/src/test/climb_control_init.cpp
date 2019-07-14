#ifndef PI
#define PI 3.1415926
#endif

#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
 
sensor_msgs::JointState actual_joint_value;
double torque_threshold = 600; 
double open_gripper = 0.04;
double close_gripper =0.0;

int main(int argc, char **argv)
{
    //function declaration
    void SubPosCallback(const sensor_msgs::JointStateConstPtr &msg);
    bool robot_reached(std::vector<double>command_pos,std::vector<double>actual_pos);   //check whether joint command_pos has arrived
    bool gripper_reached(std::vector<double>command_pos,std::vector<double>actual_pos);     //check whether gripper command_pos has arrived
    bool gripper_clamping(std::vector<double>actual_torque);    //check whether gripper  has clamped
    //end

    //ros init
    ros::init(argc, argv, "publish_joint_command");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_command", 10);
    ros::Subscriber Sub_acutal_pos = nh.subscribe("joint_states", 10, SubPosCallback);//topic come from canopen motor node
    while(Sub_acutal_pos.getNumPublishers()<1){
        std::cout<<"topic joint_states not exist!!"<<std::endl;
        sleep(1);
    }
    //end
    
    //define the joint value to publish
    sensor_msgs::JointState pub_joint_value;
    while(!nh.hasParam("climbot5d/joint_names")){
        ROS_INFO("Waiting for parameter '/climbot5d/joint_names'");
        sleep(1);
    }
    std::vector<std::string> joint_names;
    ROS_INFO("Get parameter '/climbot5d/joint_names'");
    nh.getParam("/climbot5d/joint_names",joint_names);

    pub_joint_value.name =joint_names;
    pub_joint_value.header.stamp = ros::Time::now(); 
    pub_joint_value.position.resize(joint_names.size());
    pub_joint_value.velocity.resize(joint_names.size());
    pub_joint_value.velocity[0] = 0.02;
    pub_joint_value.velocity[1] = 0.02;
    pub_joint_value.velocity[2] = 0.02;
    pub_joint_value.velocity[3] = 0.02;
    pub_joint_value.velocity[4] = 0.02;
    pub_joint_value.velocity[5] = 0.02;
    pub_joint_value.velocity[6] = 0.02; 
    //end

    ros::Rate timer(20);

    //Because of the robot power failure,the robot will lost zero positon.
    //so we should to change the following joint positions(rad) to reach the zero position.
    pub_joint_value.position[0] = open_gripper;
    pub_joint_value.position[1] = 0.0;
    pub_joint_value.position[2] = 0.0;
    pub_joint_value.position[3] = 0.0;
    pub_joint_value.position[4] = 0.0;
    pub_joint_value.position[5] = 0.0;
    pub_joint_value.position[6] = open_gripper;

    if(ros::ok()){

        //Initialize the robot to zero position
        while(ros::ok() && !robot_reached(pub_joint_value.position,actual_joint_value.position)){
        pub.publish(pub_joint_value);
        timer.sleep();
        }

        //Initialize the gripper to clamp the pole
        pub_joint_value.position[6] = close_gripper;
        pub_joint_value.position[0] = close_gripper;
        while(ros::ok() && !gripper_reached(pub_joint_value.position,actual_joint_value.position)){
            pub.publish(pub_joint_value);
            if(gripper_clamping(actual_joint_value.effort))
                break;
            timer.sleep();
        }
        timer.sleep();
    }
    return 0;
}

bool gripper_clamping(std::vector<double>actual_torque){
    bool clamping = false;
    if(actual_torque[0]>torque_threshold && actual_torque[6]>torque_threshold)
        clamping =true;
    return clamping;
}

bool gripper_reached(std::vector<double>command_pos,std::vector<double>actual_pos){
    bool reached = true;
    double eps = (0.5/180.0)*PI;
    if(abs(command_pos[0]-actual_pos[0]) > eps && abs(command_pos[6] - actual_pos[6]) > eps )
        reached = false;
    return reached;    
}

bool robot_reached(std::vector<double>command_pos,std::vector<double>actual_pos){
    bool reached = true;
    double eps = (0.5/180.0)*PI;
    for(int i = 0 ; i < command_pos.size(); i++){
        if(abs(command_pos[i]-actual_pos[i])>eps)
            reached = false;
    }
    return reached;
}

void SubPosCallback(const sensor_msgs::JointStateConstPtr &msg){
    actual_joint_value = *msg;
}