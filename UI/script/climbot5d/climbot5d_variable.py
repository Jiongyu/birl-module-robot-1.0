#!/usr/bin/env python
# -*- coding: utf-8 -*-

class Climbot5d_Variable(object):

    def __init__(self):

        # 默认速度
        self.joint_velocity = 0.02
        # 默认关节力矩
        self.open_gripper = 1000
        self.close_gripper = -1000
        self.zero_gripper = 0

        self.descartes_velocity = 0.002
