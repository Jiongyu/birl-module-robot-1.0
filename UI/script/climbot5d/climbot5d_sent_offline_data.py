#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5.QtCore import QThread,pyqtSignal

import rospy 
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from std_msgs.msg import Float64
import numpy as np 


class Climbot5d_interpolation(QThread):

    sin_interpolation_data = pyqtSignal(list)

    def __init__(self):
        super(Climbot5d_interpolation,self).__init__()
        self.offline_data = JointTrajectory()
        self.interpolation_data = JointTrajectory()
        self.max_joint_vel = Float64()

        self.pub = rospy.Publisher('climbot5d_offline_data', JointTrajectory , queue_size = 1)
        self.pub_vel = rospy.Publisher('climbot5d_max_joint_vel',Float64,queue_size = 1)
        self.sub = rospy.Subscriber('climbot5d_interpolation_data', JointTrajectory , self.interpolation_data_callback)

    def run(self):

        # sent max velocity
        self.max_joint_vel.data = self.vel
        self.pub_vel.publish(self.max_joint_vel)

        # sent offline data
        row = np.array(self.joint_value).shape[0]
        # print row
        for i in range(row):
            self.offline_data.points.append(JointTrajectoryPoint())
        for i in range(row):
            self.offline_data.points[i].positions = self.joint_value[i]
        # print self.offline_data.points
        self.pub.publish(self.offline_data)

        try:
            rospy.wait_for_message("climbot5d_interpolation_data",JointTrajectory,0.5)
            for i in range(row):
                self.offline_data.points.pop()
            self.joint_value = [[0]] * len(self.interpolation_data.points)
            for i in range(len(self.interpolation_data.points)):
                self.joint_value[i] = list(self.interpolation_data.points[i].positions)
                self.joint_value[i].extend(list(self.interpolation_data.points[i].velocities))
                self.joint_value[i].append(self.interpolation_data.points[i].effort[0])
                print self.joint_value[i]
            
            self.sin_interpolation_data.emit(self.joint_value)
        except rospy.exceptions.ROSException:
            pass

    def receive_offline(self,data):
        self.joint_value = data

    def receive_max_joint_vel(self,data):
        self.vel = data

    def interpolation_data_callback(self,msg):
        self.interpolation_data = msg