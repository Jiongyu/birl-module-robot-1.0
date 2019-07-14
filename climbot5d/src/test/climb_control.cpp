#include "./../kinematics/Kine.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Bool.h>
#include <birl_module_robot/Trajectory_point.h>
#include <ros/ros.h>
#include <math.h>
    
sensor_msgs::JointState actual_joint_value;
std_msgs::Bool gripper_is_open;
//double torque_threshold = 600; 
//double open_gripper = 0.04;
//double close_gripper =0.0;
double New_joint_value[5];//unit:degree

int main(int argc, char **argv)
{
    //function declaration
    void Round_off(double (&joint_value)[5]); //round off joint value which smaller 1e-6 and convert the rad to dgree(unit:degree)
    void SubPosCallback(const sensor_msgs::JointStateConstPtr &msg);  
    void SubGriReachCallback(const std_msgs::BoolConstPtr &msg);
    bool robot_reached(std::vector<double>command_pos,std::vector<double>actual_pos);   //check whether joint command_pos has arrived
    void Current_joint_Rad_to_Deg(double (&joint_value)[5]);  //convert the rad to deg
    //end

    ///ros init
    ros::init(argc, argv, "publish_joint_command");
    ros::NodeHandle nh;
    //control the joint 
    ros::Publisher pub_joint = nh.advertise<sensor_msgs::JointState>("joint_command", 1);
    ros::Publisher pub_traj_reached = nh.advertise<std_msgs::Bool>("traj_reached", 1);
    // control the gripper
    ros::Publisher pub_gripper_control = nh.advertise<std_msgs::Int8MultiArray>("/gripper_control_signal",1);    
    //For build a close control this topic receive joint msg come from canopen motor node.
    ros::Subscriber Sub_acutal_pos = nh.subscribe("joint_states", 1, SubPosCallback);
    while(Sub_acutal_pos.getNumPublishers()<1){
        ROS_INFO("topic joint_states not exist!!");
        sleep(1);
    }
    //detect gripper wether open or close
    gripper_is_open.data = false;
    ros::Subscriber Sub_gripper_reached = nh.subscribe("/gripper_is_open",1,SubGriReachCallback);
    while(Sub_gripper_reached.getNumPublishers()<1){
        ROS_INFO("topic gripper_is_open not exist!!");
        sleep(1);
    }
    //Get interpolation  point of trajectory
    ros::ServiceClient Client_traj_point = nh.serviceClient<birl_module_robot::Trajectory_point>("get_traj_point");
    while(Client_traj_point.waitForExistence()){
        ROS_INFO("serive get_traj_point not exist!!");
        sleep(1);
    }
    //Define the sevice to get the trajectory point
    birl_module_robot::Trajectory_point Srv_get_traj; 
    ///end
    
    //define the joint value to publish
    sensor_msgs::JointState msg_joint_value;
    while(!nh.hasParam("/climbot5d/joint_names")){
        ROS_INFO("Waiting for parameter '/climbot5d/joint_names'");
        sleep(1);
    }
    std::vector<std::string> joint_names;
    ROS_INFO("Get parameter '/climbot5d/joint_names'");
    nh.getParam("/climbot5d/joint_names",joint_names);

    msg_joint_value.name.push_back(joint_names[1]);
    msg_joint_value.name.push_back(joint_names[2]);
    msg_joint_value.name.push_back(joint_names[3]);
    msg_joint_value.name.push_back(joint_names[4]);
    msg_joint_value.name.push_back(joint_names[5]);
    msg_joint_value.header.stamp = ros::Time::now(); 
    msg_joint_value.position.resize(joint_names.size()-2);
    msg_joint_value.velocity.resize(joint_names.size()-2);
    msg_joint_value.velocity[0] = 0.02;
    msg_joint_value.velocity[1] = 0.02;
    msg_joint_value.velocity[2] = 0.02;
    msg_joint_value.velocity[3] = 0.02;
    msg_joint_value.velocity[4] = 0.02;
    //end

    //define the msg to control gripper
    std_msgs::Int8MultiArray msg_gripper_open;
    msg_gripper_open.data.resize(2);
    /*
        msg_gripper_open.data[0]: gripper(ID) gripper0(0) gripper6(6)
        msg_gripper_open.data[1]: open(1) close(0)
    */ 
    //end
    
    ///kine init
    double Robot_Link_Len[6] = {0.1764,0.2568,0.2932,0.2932,0.2568,0.1764}; //robot link length
    Kine_CR_FiveDoF_G1 Climbot_G1; // robot based on the gripper1 to get inverse solution
    Kine_CR_FiveDoF_G2 Climbot_G2; // robot based on the gripper6 to get inverse solution
    Climbot_G1.Set_Length(Robot_Link_Len);
    Climbot_G2.Set_Length(Robot_Link_Len);
    double New_point[6]; //new cartesian point (xyzwpr) unit:(meter,degree)
    double Current_joint_value[5] = {0,0,0,0,0};  //unit:degree
    ///end

    bool G0_gripper_open = false;
    ros::Rate timer(20);
    while(ros::ok()){
        ///chose which is base and open another
        if(!G0_gripper_open){
            //base on G0 and open G6
            msg_gripper_open.data[0] = 6;
            msg_gripper_open.data[1] = 1;
        }//end
        else{
            //base on G6 and open G0
            msg_gripper_open.data[0] = 0;
            msg_gripper_open.data[1] = 1; 
        }
        pub_gripper_control.publish(msg_gripper_open);
        while(!gripper_is_open.data)
        {
            ROS_INFO("gripper not open.");
            ros::spinOnce();
            sleep(0.5);
        }
        ///end

        ///publish trajectory
        Srv_get_traj.request.get_point = true;
        while(1){
            if(!Client_traj_point.call(Srv_get_traj)){
                ROS_INFO("Do not receive Trajectory point message.");
                sleep(1);
            }
            else break;
        }
        std_msgs::Bool msg_traj_reached;
        msg_traj_reached.data = false;
        pub_traj_reached.publish(msg_traj_reached);
        int index = 0;
        int num_traj_point = Srv_get_traj.response.tra_joint_point.size() - 1;
        while(index != num_traj_point){

            New_point[0] =  Srv_get_traj.response.tra_joint_point[index];  
            New_point[1] =  Srv_get_traj.response.tra_joint_point[index+1]; 
            New_point[2] =  Srv_get_traj.response.tra_joint_point[index+2];
            New_point[3] =  Srv_get_traj.response.tra_joint_point[index+3];
            New_point[4] =  Srv_get_traj.response.tra_joint_point[index+4];
            New_point[5] =  Srv_get_traj.response.tra_joint_point[index+5]; 
            index += 6;

            Current_joint_Rad_to_Deg(Current_joint_value);
            if(!G0_gripper_open){
                //Given the new cartesian point and current joint value this function get the inverse solution base on the G1_Joint
                Climbot_G1.IKine(New_point,Current_joint_value,New_joint_value); 
            }
            else{
                //Given the new cartesian point and current joint value this function get the inverse solution base on the G6_Joint
                Climbot_G2.IKine(New_point,Current_joint_value,New_joint_value);                  
            }
            Round_off(New_joint_value);
            msg_joint_value.position[0] = New_joint_value[0]; //I1_Joint
            msg_joint_value.position[1] = New_joint_value[1]; //T2_Joint
            msg_joint_value.position[2] = New_joint_value[2]; //T3_Jonit
            msg_joint_value.position[3] = New_joint_value[3]; //T4_Joint
            msg_joint_value.position[4] = New_joint_value[4]; //I5_Joint

            pub_joint.publish(msg_joint_value);
            ros::spinOnce();
            while(!robot_reached(msg_joint_value.position,actual_joint_value.position)){
                pub_joint.publish(msg_joint_value);
                sleep(0.5);
            }
        }
        msg_traj_reached.data = true;
        pub_traj_reached.publish(msg_traj_reached);
        //Current_joint_value = New_joint_value
        for(int i = 0 ; i < 5 ; i ++){
            Current_joint_value[i] = New_joint_value[i];
        }//end
        ///end

        //close_gripper gripper6 or gripper0
        if(!G0_gripper_open){
            msg_gripper_open.data[0] = 6;
            msg_gripper_open.data[1] = 0;
        }
        else{
            msg_gripper_open.data[0] = 0;
            msg_gripper_open.data[1] = 0;        
            ros::spinOnce();
        }
        pub_gripper_control.publish(msg_gripper_open);
        while(gripper_is_open.data)
        {
            ROS_INFO("gripper not close.");
            ros::spinOnce();
            sleep(0.5);
        }
        //end

        G0_gripper_open = !G0_gripper_open;
        timer.sleep();
    }
    return 0;
}

 //round off joint value which smaller 1e-6 and convert the rad to dgree
void Round_off(double (&joint_value)[5]){
    for(int i = 0;i < 5; i++){
        if(joint_value[i]<1e-6)
            joint_value[i] = 0.0;
        else joint_value[i]*= PI_RAD;
    }
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

void SubGriReachCallback(const std_msgs::BoolConstPtr &msg){
    gripper_is_open = *msg;
}

void Current_joint_Rad_to_Deg(double (&joint_value)[5]){
    joint_value[0]*=PI_DEG;
    joint_value[1]*=PI_DEG;
    joint_value[2]*=PI_DEG;
    joint_value[3]*=PI_DEG;
    joint_value[4]*=PI_DEG;
}