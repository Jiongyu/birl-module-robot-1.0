#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy 
from math import degrees
from std_msgs.msg import Float64MultiArray 

joint_point = Float64MultiArray()

def main():
    global joint_point
    rospy.init_node('get_Data',anonymous=True)
    pub = rospy.Publisher('climbot5d_Descartes_point',Float64MultiArray,queue_size=1)
    sub = rospy.Subscriber('climbot5d_Inverse_solution',Float64MultiArray,callback)

    descart_point = Float64MultiArray()
    descart_point.data = [0] * 18
    descart_point.data[0] = 0

    descart_point.data[1] = 0.5864
    descart_point.data[2] = 0
    descart_point.data[3] = 0
    descart_point.data[4] = 0
    descart_point.data[5] = 0
    descart_point.data[6] = 3.141592654

    descart_point.data[7] = 0
    descart_point.data[8] = 0
    descart_point.data[9] = 0
    descart_point.data[10] = 0
    descart_point.data[11] = 0

    descart_point.data[12] = -0.02
    descart_point.data[13] = 0
    descart_point.data[14] = 0
    descart_point.data[15] = 0
    descart_point.data[16] = 0
    descart_point.data[17] = 0

    joint_point.data = [0] * 10

    dx = 0.005
    f = open('get_joint_data.txt','w')
    while not rospy.is_shutdown() and descart_point.data[1] >= 0.3864:
        pub.publish(descart_point)
        descart_point.data[1] -= dx
        rospy.wait_for_message('climbot5d_Inverse_solution',Float64MultiArray)
        f.write('pos:{0},{1},{2},{3},{4}\n'.format(joint_point.data[0],joint_point.data[1],\
                                                            joint_point.data[2],joint_point.data[3],\
                                                            joint_point.data[4]))
        f.write('vel:{0},{1},{2},{3},{4}\n'.format(joint_point.data[5],joint_point.data[6],\
                                                            joint_point.data[7],joint_point.data[8],\
                                                            joint_point.data[9]))
        f.write('\n')
                                                            
        # descart_point.data[7] = joint_point.data[0]
        # descart_point.data[8] = joint_point.data[1]
        # descart_point.data[9] = joint_point.data[2]
        # descart_point.data[10] = joint_point.data[3]
        # descart_point.data[11] = joint_point.data[4]

    f.close()


def callback(msg):
    global joint_point
    joint_point = msg



main()

        




        