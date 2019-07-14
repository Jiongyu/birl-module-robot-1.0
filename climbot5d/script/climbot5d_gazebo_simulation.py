#!/usr/bin/env python
# -*- coding: utf-8 -*-
import traceback
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import JointState
import time
from math import fabs

receive_joint_command = JointState()

def main():
    global receive_joint_command
    rospy.init_node("gazebo_publish_joint",anonymous=True)
    pub_joint_command = rospy.Publisher("/climbot5d/joint_position_controller/command",Float64MultiArray,queue_size=10)
    pub_gripper_G0_command = rospy.Publisher("/climbot5d/jointG0_effort_controller/command",Float64,queue_size=10)
    pub_gripper_G01_command = rospy.Publisher("/climbot5d/jointG0_1_effort_controller/command",Float64,queue_size=10)
    pub_gripper_G6_command = rospy.Publisher("/climbot5d/jointG6_effort_controller/command",Float64,queue_size=10)
    pub_gripper_G61_command = rospy.Publisher("/climbot5d/jointG6_1_effort_controller/command",Float64,queue_size=10)
    pub_model_state = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size=10)

    ########攀爬位置初始化#######################################################
    init_joint_data = Float64MultiArray()
    init_joint_data.data = [0,-1.570796327,0,1.570796327,0]  # I1 T2 T3 T4 I5

    G0 = Float64()
    G01 = Float64()
    G6 = Float64()
    G61 = Float64()
    G0.data = 100
    G01.data = 100
    G6.data = 100
    G61.data = 100

    robot = ModelState()
    robot.model_name = "climbot5d"
    robot.pose.position.x = 0
    robot.pose.position.y = 0   
    robot.pose.position.z = 1
    robot.pose.orientation.x = 0
    robot.pose.orientation.y = 0
    robot.pose.orientation.z = 0
    robot.pose.orientation.w = 1
    robot.twist.linear.x = 0
    robot.twist.linear.y = 0
    robot.twist.linear.z = 0
    robot.twist.angular.x = 0
    robot.twist.angular.y = 0
    robot.twist.angular.z = 0
    
    for i in range(30):
        pub_gripper_G0_command.publish(G0)
        pub_gripper_G01_command.publish(G01)
        pub_gripper_G6_command.publish(G6)
        pub_gripper_G61_command.publish(G61)
        pub_joint_command.publish(init_joint_data)
        pub_model_state.publish(robot)
        time.sleep(0.1)

    G0.data = -100
    G01.data = -100
    G6.data = -100
    G61.data = -100

    for i in range(30):
        pub_gripper_G0_command.publish(G0)
        pub_gripper_G01_command.publish(G01)
        pub_gripper_G6_command.publish(G6)
        pub_gripper_G61_command.publish(G61)
        pub_joint_command.publish(init_joint_data)
        time.sleep(0.1)

    ###################################################################
    sub_command = rospy.Subscriber("/climbot5d_simulation_joint_command",JointState,callback)
    receive_joint_command.name = ["G0_Joint", "I1_Joint", "T2_Joint", "T3_Joint", "T4_Joint", "I5_Joint", "G6_Joint"]
    receive_joint_command.position = [-100,0,0,0,0,0,-100]
    current_joint_state = [-100,0,-1.570796327,0,1.570796327,0,-100]
    joint_command = Float64MultiArray()

    timer = rospy.Rate(30)
    while(not rospy.is_shutdown()):
        """
        print receive_joint_command.position[0]
        print receive_joint_command.position[1]
        print receive_joint_command.position[2]
        print receive_joint_command.position[3]
        print receive_joint_command.position[4]
        print receive_joint_command.position[5]
        print receive_joint_command.position[6]
        print "-------------------------------------------"
        """
        if( (receive_joint_command.position[0] != current_joint_state[0]) or
            (receive_joint_command.position[1] != current_joint_state[1]) or
            (receive_joint_command.position[2] != current_joint_state[2]) or
            (receive_joint_command.position[3] != current_joint_state[3]) or 
            (receive_joint_command.position[4] != current_joint_state[4]) or
            (receive_joint_command.position[5] != current_joint_state[5]) or 
            (receive_joint_command.position[6] != current_joint_state[6]) ):

            joint_command.data = [receive_joint_command.position[1],-1.57+receive_joint_command.position[2],\
                                receive_joint_command.position[3],1.57+receive_joint_command.position[4],\
                                receive_joint_command.position[5]]

            G0.data = receive_joint_command.position[0]
            G01.data = receive_joint_command.position[0]
            G6.data = receive_joint_command.position[6]
            G61.data = receive_joint_command.position[6]

            pub_gripper_G0_command.publish(G0)
            pub_gripper_G01_command.publish(G01)
            pub_gripper_G6_command.publish(G6)
            pub_gripper_G61_command.publish(G61)
            pub_joint_command.publish(joint_command)

            current_joint_state[0] = receive_joint_command.position[0]
            current_joint_state[1] = receive_joint_command.position[1]
            current_joint_state[2] = receive_joint_command.position[2]
            current_joint_state[3] = receive_joint_command.position[3]
            current_joint_state[4] = receive_joint_command.position[4]
            current_joint_state[5] = receive_joint_command.position[5]
            current_joint_state[6] = receive_joint_command.position[6]
            print "got new message!"
        else:
            pub_joint_command.publish(joint_command)
        timer.sleep()

def callback(msg):
    global receive_joint_command
    receive_joint_command = msg
    #print receive_joint_command.position[0]

if __name__ == "__main__":
    try:
        main()
    except Exception, e:
        print 'str(Exception):\t', str(Exception) 
        print 'str(e):\t\t', str(e)
        traceback.print_exc()