#include "include/change_base_link.h"

sensor_msgs::JointState robot_joint_state;
geometry_msgs::TransformStamped T;

//callback function about joint state 
void pose_callback(const sensor_msgs::JointState::ConstPtr& msg){
    robot_joint_state = *msg;
}
// callback function about transform of gripper A to world 
void transform_callback(const geometry_msgs::TransformStamped::ConstPtr& tran){
    T = * tran;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_joint_state");
    ros::NodeHandle nh;
    Change_base_link climbot5d;

    std::vector<std::string> joint_names;
    joint_names = {"G0_Joint", "I1_Joint", "T2_Joint", "T3_Joint", "T4_Joint", "I5_Joint", "G6_Joint"};
    robot_joint_state.name = joint_names;
    robot_joint_state.position.resize(joint_names.size());
    robot_joint_state.velocity.resize(joint_names.size()); 
    ros::Rate timer(40);

    tf::TransformBroadcaster br;
    tf::Transform transform;
  
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/climbot5d/joint_command", 20);
    ros::Subscriber sub_transform = nh.subscribe("/climbot5d/transform", 20, transform_callback);
    ros::Subscriber sub_pose = nh.subscribe("/climbot5d/simulation_pub_joint_command", 20, pose_callback);

    while(ros::ok()){
        ROS_INFO("Translation,Rotation:");
        ROS_INFO("%f,%f,%f",T.transform.translation.x,T.transform.translation.y,T.transform.translation.z);
        ROS_INFO("%f,%f,%f,%f",T.transform.rotation.x,T.transform.rotation.y,T.transform.rotation.z,T.transform.rotation.w);
        //publish transform A to world
        transform.setOrigin(tf::Vector3(T.transform.translation.x,T.transform.translation.y,T.transform.translation.z));
        transform.setRotation( tf::Quaternion(T.transform.rotation.x,T.transform.rotation.y,T.transform.rotation.z,T.transform.rotation.w));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/base_gripper1"));
        //publish joint value
        pub.publish(robot_joint_state);     
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}