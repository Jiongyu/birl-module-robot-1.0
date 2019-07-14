#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_control_test");
    ros::NodeHandle nh;
    ros::Publisher pub_gripper = nh.advertise<sensor_msgs::JointState>("gripper_command", 100);
    sensor_msgs::JointState  gripper_command;
    gripper_command.position.resize(1);
    gripper_command.velocity.resize(1);
    //gripper_command.velocity = {0.02};
    gripper_command.name.resize(1);
    gripper_command.name ={"G0_Joint"};
    
    double key_value = 0.0;
    double dk = 0.005;
    ros::Rate timer(40);

    while (ros::ok()&&key_value<=1)
    {
        key_value+=dk;
        gripper_command.position.push_back(key_value);
        gripper_command.velocity.push_back(0.02);
        pub_gripper.publish(gripper_command);
        gripper_command.velocity.clear();
        gripper_command.position.clear();
        timer.sleep();
    }
    
    
    return 0;
}