#include "./../kinematics/Kine.h"
#include "./../kinematics/Interpolation.h"
#include "ros/ros.h"
#include "std_msgs/Int64MultiArray.h"
#include "./../kinematics/MyPath.h"
#include <vector>
#include <deque>

int main(int argc, char **argv)
{
    Interpolation Descartes_interpolation;
    double Robot_Link_Len[6] = {0.1764,0.2568,0.2932,0.2932,0.2568,0.1764}; //robot link length
    Descartes_interpolation.Set_Length(Robot_Link_Len);
    double rat[5] = {457,480,480,480,457};
    Descartes_interpolation.Set_Rat(rat);
    // 位置极限设置根据攀爬初始关节位置设计
    double pos[5] = {360,30,120,30,360};
    double neg[5] = {-360,-210,-120,-210,-360};
    Descartes_interpolation.Set_Lim(pos,neg);

    InterpStruct input_11,input_22;
    input_11.Mode = 1;
    input_22.Mode = 2;

    for(int i=0;i<5;i++)
        input_11.JStart[i] = 0;

    // input_11.JMid[0] = 0;
    // input_11.JMid[1] = 10;
    // input_11.JMid[2] = 20;
    // input_11.JMid[3] = 10;
    // input_11.JMid[4] = 0;

    input_11.JEnd[0] = 0;
    input_11.JEnd[1] = -2;
    input_11.JEnd[2] = 4;
    input_11.JEnd[3] = -2;
    input_11.JEnd[4] = 0;

    input_11.Acc = 2;
    input_11.Jerk = 1;
    input_11.Vel = 5;
    input_11.G_V = 10;
    input_11.T_Acc = 0.1;

    input_22.JStart[0] = 0;
    input_22.JStart[1] = -50;
    input_22.JStart[2] = 100;
    input_22.JStart[3] = -50;
    input_22.JStart[4] = 0;

    input_22.JEnd[0] = 0;
    input_22.JEnd[1] = -60;
    input_22.JEnd[2] = 120;
    input_22.JEnd[3] = -60;
    input_22.JEnd[4] = 0;

    input_22.Acc = 2;
    input_22.Jerk = 1;
    input_22.Vel = 5;
    input_22.G_V = 10;
    input_22.T_Acc = 0.1;

//     std::cout<<"------------------line_plan-----------------------------------"<<std::endl;
//     Descartes_interpolation.Plan_Line(&input);

//     for(size_t i = 0;i<Descartes_interpolation.outputdata.size();i++){
//         std::cout<<"Position: "<< Descartes_interpolation.outputdata[i].P[0]<<','<<Descartes_interpolation.outputdata[i].P[1]<<','<<Descartes_interpolation.outputdata[i].P[2]<<','<<Descartes_interpolation.outputdata[i].P[3]<<','<<Descartes_interpolation.outputdata[i].P[4]<<std::endl;
//         std::cout<<"velocity: "<< Descartes_interpolation.outputdata[i].V[0]<<','<<Descartes_interpolation.outputdata[i].V[1]<<','<<Descartes_interpolation.outputdata[i].V[2]<<','<<Descartes_interpolation.outputdata[i].V[3]<<','<<Descartes_interpolation.outputdata[i].V[4]<<std::endl;
//         std::cout<<"time: "<<Descartes_interpolation.outputdata[i].T<<std::endl;
//         std::cout<<"--------------------------------------"<<std::endl;;
//    }
//     std::cout<<"output size: "<<Descartes_interpolation.outputdata.size()<<std::endl;
//     std::cout<<"--------------------------------------------------------------"<<std::endl;


    JointInterpStruct input_1,input_2,input_3;

    Descartes_interpolation.Plan_JointS(&input_11,&input_1);
    // Descartes_interpolation.Plan_JointS(&input_22,&input_2);

    std::cout<<"start: "<<input_1.JStart[0]<<","<<input_1.JStart[1]<<","<<input_1.JStart[2]<<","<<input_1.JStart[3]<<","<<input_1.JStart[4]<<std::endl;
    std::cout<<"end: "<<input_1.JEnd[0]<<","<<input_1.JEnd[1]<<","<<input_1.JEnd[2]<<","<<input_1.JEnd[3]<<","<<input_1.JEnd[4]<<std::endl;
    std::cout<<"vel: "<<input_1.Vel[0]<<","<<input_1.Vel[1]<<","<<input_1.Vel[2]<<","<<input_1.Vel[3]<<","<<input_1.Vel[4]<<std::endl;
    std::cout<<"T_Acc: "<<input_1.T_Acc<<std::endl;
    std::cout<<"T: "<<input_1.T<<std::endl;

//     input_1.JStart[0] = 0;
//     input_1.JStart[1] = 0;
//     input_1.JStart[2] = 0;
//     input_1.JStart[3] = 0;
//     input_1.JStart[4] = 0;
//     input_1.JEnd[0] = 0;
//     input_1.JEnd[1] = 10;
//     input_1.JEnd[2] = 20;
//     input_1.JEnd[3] = 10;
//     input_1.JEnd[4] = 0;
//     input_1.Vel[0] = 0;
//     input_1.Vel[1] = 5;
//     input_1.Vel[2] = 10;
//     input_1.Vel[3] = 5;
//     input_1.Vel[4] = 0;
//     input_1.T_Acc = 0.1;
//     input_1.T = 4;

//     input_2.JStart[0] = 0;
//     input_2.JStart[1] = 20;
//     input_2.JStart[2] = 40;
//     input_2.JStart[3] = 20;
//     input_2.JStart[4] = 0;
//     input_2.JEnd[0] = 0;
//     input_2.JEnd[1] = 30;
//     input_2.JEnd[2] = 60;
//     input_2.JEnd[3] = 30;
//     input_2.JEnd[4] = 0;
//     input_2.Vel[0] = 0;
//     input_2.Vel[1] = 5;
//     input_2.Vel[2] = 10;
//     input_2.Vel[3] = 5;
//     input_2.Vel[4] = 0;
//     input_2.T_Acc = 0.1;
//     input_2.T = 4;

//     input_3.JStart[0] = 0;
//     input_3.JStart[1] = 40;
//     input_3.JStart[2] = 80;
//     input_3.JStart[3] = 40;
//     input_3.JStart[4] = 0;
//     input_3.JEnd[0] = 0;
//     input_3.JEnd[1] = 50;
//     input_3.JEnd[2] = 100;
//     input_3.JEnd[3] = 50;
//     input_3.JEnd[4] = 0;
//     input_3.Vel[0] = 0;
//     input_3.Vel[1] = 5;
//     input_3.Vel[2] = 10;
//     input_3.Vel[3] = 5;
//     input_3.Vel[4] = 0;
//     input_3.T_Acc = 0.1;
//     input_3.T = 4;

    std::deque<JointInterpStruct> joint_point;
    joint_point.clear();
    joint_point.push_back(input_1);
    // joint_point.push_back(input_2);
    // joint_point.push_back(input_3);

    Descartes_interpolation.outputdata.clear();
    Descartes_interpolation.Plan_JointM(&joint_point);
    for(size_t i = 0;i<Descartes_interpolation.outputdata.size();i++){
        std::cout<<"Position: "<< Descartes_interpolation.outputdata[i].P[0]<<','<<Descartes_interpolation.outputdata[i].P[1]<<','<<Descartes_interpolation.outputdata[i].P[2]<<','<<Descartes_interpolation.outputdata[i].P[3]<<','<<Descartes_interpolation.outputdata[i].P[4]<<std::endl;
        std::cout<<"velocity: "<< Descartes_interpolation.outputdata[i].V[0]<<','<<Descartes_interpolation.outputdata[i].V[1]<<','<<Descartes_interpolation.outputdata[i].V[2]<<','<<Descartes_interpolation.outputdata[i].V[3]<<','<<Descartes_interpolation.outputdata[i].V[4]<<std::endl;
        std::cout<<"time: "<<Descartes_interpolation.outputdata[i].T<<std::endl;
        std::cout<<"--------------------------------------"<<std::endl;
   }
    std::cout<<"output size: "<<Descartes_interpolation.outputdata.size()<<std::endl;

    return 0;
}