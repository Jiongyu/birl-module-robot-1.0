#define PI_RAD  0.0174532925199 // 角度转换为弧度参数
#include "./../kinematics/Kine.h"
#include "./../kinematics/Interpolation.h"
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <deque>

trajectory_msgs::JointTrajectory offline_data;
std_msgs::Float64 max_velocity;
bool new_velocity;
bool new_offline_data;

int main(int argc, char **argv)
{
    void subCallback(const trajectory_msgs::JointTrajectoryConstPtr &msg);
    void subVelCallback(const std_msgs::Float64ConstPtr &msg);

    // 插补初始化
    Interpolation Descartes_interpolation;
    double Robot_Link_Len[6] = {0.1764,0.2568,0.2932,0.2932,0.2568,0.1764}; //robot link length
    Descartes_interpolation.Set_Length(Robot_Link_Len);
    double rat[5] = {457,480,480,480,457};
    Descartes_interpolation.Set_Rat(rat);
    // 位置极限设置根据攀爬初始关节位置设计
    double pos[5] = {360,30,120,30,360};
    double neg[5] = {-360,-210,-120,-210,-360};
    Descartes_interpolation.Set_Lim(pos,neg);

    InterpStruct input;
    JointInterpStruct input_1;
    std::deque<JointInterpStruct> joint_point;
    joint_point.clear();

    new_velocity = false;
    new_offline_data = false;

    // ros 初始化
    ros::init(argc, argv, "generate_interpolation_data");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("climbot5d_interpolation_data", 5);
    ros::Subscriber sub = nh.subscribe("climbot5d_offline_data", 5, subCallback);

    while(ros::ok() && sub.getNumPublishers()<1){
        ROS_INFO("Topic offline_data not exist!!");
        sleep(1);
    }

    ros::Subscriber sub_vel = nh.subscribe("climbot5d_max_joint_vel", 5, subVelCallback);
    while(ros::ok() && sub_vel.getNumPublishers()<1){
        ROS_INFO("Topic climbot5d_max_joint_vel not exist!!");
        sleep(1);
    }  

    trajectory_msgs::JointTrajectory interpolation_data;
    trajectory_msgs::JointTrajectoryPoint interpolation_data_points;

    int number_offline_data;

         input.Mode = 1;
        input.Acc = 4;
        input.Jerk = 1;
        input.Vel = 1.1;
        input.G_V = 1.1;
        input.T_Acc = 0.03;

    while(ros::ok()){
        // 轨迹参数
        if (new_velocity){
            input.Mode = 1;
            input.Acc = 4;
            input.Jerk = 1;
            input.Vel = max_velocity.data;
            input.G_V = max_velocity.data;
            input.T_Acc = 0.03;
            new_velocity = false;
        }

        if (new_offline_data){

            number_offline_data = offline_data.points.size();
            std::cout<<"离线数据点数:"<<number_offline_data<<std::endl;
            
            int i = 0;
            while(i < number_offline_data-1){

                input.JStart[0] = offline_data.points[i].positions[0];
                input.JStart[1] = offline_data.points[i].positions[1];
                input.JStart[2] = offline_data.points[i].positions[2];
                input.JStart[3] = offline_data.points[i].positions[3];
                input.JStart[4] = offline_data.points[i].positions[4];
                // std::cout<<input.JStart[1]<<std::endl;

                input.JEnd[0] = offline_data.points[i+1].positions[0];
                input.JEnd[1] = offline_data.points[i+1].positions[1];
                input.JEnd[2] = offline_data.points[i+1].positions[2];
                input.JEnd[3] = offline_data.points[i+1].positions[3];
                input.JEnd[4] = offline_data.points[i+1].positions[4];
                // std::cout<<input.JEnd[1]<<std::endl;

                i+=1;
                Descartes_interpolation.Plan_JointS(&input,&input_1);
                // std::cout<<"start: "<<input_1.JStart[0]<<","<<input_1.JStart[1]<<","<<input_1.JStart[2]<<","<<input_1.JStart[3]<<","<<input_1.JStart[4]<<std::endl;
                // std::cout<<"end: "<<input_1.JEnd[0]<<","<<input_1.JEnd[1]<<","<<input_1.JEnd[2]<<","<<input_1.JEnd[3]<<","<<input_1.JEnd[4]<<std::endl;
                // std::cout<<"vel: "<<input_1.Vel[0]<<","<<input_1.Vel[1]<<","<<input_1.Vel[2]<<","<<input_1.Vel[3]<<","<<input_1.Vel[4]<<std::endl;
                // std::cout<<"T_Acc: "<<input_1.T_Acc<<std::endl;
                // std::cout<<"T: "<<input_1.T<<std::endl;
                joint_point.push_back(input_1);
            }
            Descartes_interpolation.outputdata.clear();
            Descartes_interpolation.Plan_JointM(&joint_point);

            int number_interpolation_data;
            number_interpolation_data = Descartes_interpolation.outputdata.size();
            std::cout<<"插值点数："<<number_interpolation_data<<std::endl;
            // for( int z=0;z<number_interpolation_data;z++){
            //     std::cout<<"t: "<<Descartes_interpolation.outputdata[z].T<<std::endl;
            // }

            i = 0;
            double max_position=0,max_velocity=0,max_data= 0;
            while(i < number_interpolation_data - 1){

                for(int j=0; j<5;j++){
                    interpolation_data_points.positions.push_back(Descartes_interpolation.outputdata[i].P[j]*PI_RAD);
                    interpolation_data_points.velocities.push_back(Descartes_interpolation.outputdata[i].V[j]*PI_RAD);
                    
                    max_data = fabs(Descartes_interpolation.outputdata[i+1].P[j] - Descartes_interpolation.outputdata[i].P[j]);
                    if (max_data > max_position){
                        max_position = max_data;
                    }
                    max_data = fabs(Descartes_interpolation.outputdata[i].V[j]);
                    if (max_data > max_velocity){
                        max_velocity = max_data;
                    }
                }
                // 使用effort代替存储时间
                if (max_position && max_velocity){
                    interpolation_data_points.effort.push_back(max_position / max_velocity - 0.005);
                }else{
                    interpolation_data_points.effort.push_back(0);
                }

                interpolation_data.points.push_back(interpolation_data_points);
                interpolation_data_points.positions.clear();
                interpolation_data_points.velocities.clear();
                interpolation_data_points.effort.clear();
                i+=1;
            }
            pub.publish(interpolation_data);
            joint_point.clear();
            interpolation_data.points.clear();
            offline_data.points.clear();
            new_offline_data = false;
        }
        ros::spinOnce();
        sleep(0.5);
    }
    return 0;
}

void subCallback(const trajectory_msgs::JointTrajectoryConstPtr &msg){
    offline_data = *msg;
    new_offline_data = true;
}

void subVelCallback(const std_msgs::Float64ConstPtr &msg){
    max_velocity = *msg;
    new_velocity = true;
}