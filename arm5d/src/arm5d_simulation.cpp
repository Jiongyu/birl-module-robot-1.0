#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

sensor_msgs::JointState joint_data;

int main(int argc, char **argv)
{
    void subCallback(const sensor_msgs::JointStateConstPtr &msg);
    ros::init(argc, argv, "arm5d_simulation");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("Arm5d_simulation_Joint_state", 10);
    ros::Subscriber sub = nh.subscribe("Arm5d_simulation_joint_command", 10, subCallback);

    while (sub.getNumPublishers() < 1)
    {
        ROS_INFO("topic Arm5d_simulation_joint_command not exist!!!!");
        sleep(1);        
    }
    sensor_msgs::JointState joint_state;
    std::vector<std::string> joint_names;
    joint_names = {"I1_Joint","T2_Joint","T3_Joint","i4_Joint","t5_Joint"};
    std::vector<std::double_t> joint_position;
    joint_position = {0,0,0,0,0};
    joint_data.position.resize(joint_position.size());
    joint_data.position = joint_position;
    joint_state.name.resize(joint_names.size());
    joint_state.name = joint_names;
    joint_state.position.resize(joint_position.size());
    joint_state.position = joint_position;

    ros::Rate timer(30);
    while(ros::ok()){
        if(joint_data.position[0] != joint_state.position[0] ||
           joint_data.position[1] != joint_state.position[1] ||
           joint_data.position[2] != joint_state.position[2] ||
           joint_data.position[3] != joint_state.position[3] ||
           joint_data.position[4] != joint_state.position[4] ){

                joint_state.position[0] = joint_data.position[0];
                joint_state.position[1] = joint_data.position[1];
                joint_state.position[2] = joint_data.position[2];
                joint_state.position[3] = joint_data.position[3];
                joint_state.position[4] = joint_data.position[4];
                pub.publish(joint_state);
           }
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}

void subCallback(const sensor_msgs::JointStateConstPtr &msg){
    joint_data = *msg;
}