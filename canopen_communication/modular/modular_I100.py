#!/usr/bin/env python2.7
# -*- coding: UTF-8 -*-
import sys
sys.path.append("./src/birl_modular_robot/canopen_communication/modular")
from  canopen_control_init import Canopen_control_init
import time
from math import radians,degrees
#import pdb

class I100(Canopen_control_init):
    """
    I85 control base on the canopen
    """
    def __init__(self,id,eds_file):
        """
        add node to the can
        :param id: node id
        :param eds_file: the location of the eds file
        :return:
        """
        super(I100, self).__init__()
        self.id = id
        self.eds_file = eds_file
        self.node = self.network.add_node(self.id, self.eds_file)

    def start(self):
        """
        start communication
        """
        self.node.nmt.state = 'RESET'
        self.node.nmt.wait_for_bootup(10)
        print('node {1} state 1) = {0}'.format(self.node.nmt.state, self.id))

        error_log = self.node.sdo[0x1003]
        for error in error_log.values():
            print("Error {0} was found in the log".format(error.raw))
        print('node {1} state 2) = {0}'.format(self.node.nmt.state, self.id))

        try:
            #pdb.set_trace()
            self._eds_configure()
        except:
            print("sdo configure error!!!")

        print('node {1} state 3) = {0}'.format(self.node.nmt.state, self.id))

        self.node.nmt.state = "OPERATIONAL"
        print('node {1} state 4) = {0}'.format(self.node.nmt.state, self.id))

    def stop(self):
        """
        stop communication
        """
        self.node.nmt.state = 'PRE-OPERATIONAL'
        print('node {1} state 5) = {0}'.format(self.node.nmt.state, self.id))
        self.network.sync.stop()
        self.network.disconnect()

    def set_mode(self, mode):
        """
        :param mode: motor operation mode(1,3,4)
        1: profiled position
        3: Profiled Velocity
        4: Profiled Torque
        """
        self.mode = mode
        if (self.mode == 1 or self.mode == 3 or self.mode == 4):
            try:
                self.node.sdo[0x6060].raw = self.mode
            except:
                print "set mode error"

    def sent_position(self,position,velocity = 0.03):
        """
        In the profile position mode this function sent some control message to motor.
        :param position: motor position(rad)
        :param velocity: default motor velocity(rad/s)
        """
        if(self.mode == 1):

            self.node.sdo[0x6081].raw = 10 * self._I100_msg_to_device(velocity)  
            self.node.sdo[0x6060].raw = 1  # profile position mode
            self.node.sdo[0x6086].raw = 0
            self.node.sdo.download(0x607a, 0x0, self._decTohex_32(self._I100_msg_to_device(position)))

            self.node.sdo[0x6040].bits[4] = 0
            self.node.sdo[0x6040].bits[4] = 1
            self.node.sdo[0x6040].bits[5] = 1
            self.node.sdo[0x6040].bits[6] = 0
            self.node.sdo[0x6040].bits[4] = 0

    def sent_velocity(self,velocity):
        """
        In the profile velocity mode this function sent some control message to motor.
        :param velocity: motor velocity(rad/s)
        :return:
        """
        if self.mode == 3:  # Profiled Velocity
            self.node.sdo[0x6040].bits[0] = 1
            self.node.sdo[0x6040].bits[1] = 1
            self.node.sdo[0x6040].bits[2] = 1
            self.node.sdo[0x6040].bits[3] = 1
            # self.node.sdo[0x6040].bits[7] = 0
            velocity = 10 * self._I100_msg_to_device(velocity)
            self.node.sdo.download(0x60ff, 0x0, self._decTohex_32(velocity))  # velocity


    def sent_torque(self,torque):
        """
        In the profile torque mode this function sent some control message to motor.
        :param torque: motor torque()
        :return:
        """
        if self.mode == 4:  # Profiled Torque
            # enable operation
            self.node.sdo[0x6040].bits[0] = 1
            self.node.sdo[0x6040].bits[1] = 1
            self.node.sdo[0x6040].bits[2] = 1
            self.node.sdo[0x6040].bits[3] = 1
            #self.node.sdo[0x6040].bits[4] = 1
            #self.node.sdo[0x6040].bits[7] = 0
            self.node.sdo.download(0x6071, 0x0,self._decTohex(torque))  # torque
            

    def get_position(self):
        """
        get the motor actual value
        :return position(rad)
        """
        return self._I100_msg_from_device(self.node.sdo[0x6064].phys)  # rad

    def get_velocity(self):
        """
        get the motor actual value
        :return velocity(rad/s)
        """
        return (self._I100_msg_from_device(self.node.sdo[0x606c].phys)) * 10 # rad/s

    def get_torque(self):
        """
        get the motor actual value
        :return torque(rate torque(mN.m) /1000)
        """
        return self.node.sdo[0x6077].phys   # rate torque(mN.m) /1000

    def get_current(self):
        """
        get the motor actual value
        :return current(mA)
        """
        return self.node.sdo[0x221c].phys  # mA

    def get_operation_mode(self):
        return self.node.sdo[0x6061].phys
        pass
        
    def quick_stop(self):
        self.node.sdo[0x6040].bits[2] = 0
        self.node.nmt.state = 'PRE-OPERATIONAL'
        print('node {1} state 5) = {0}'.format(self.node.nmt.state, self.id))
        self.network.sync.stop()
        self.network.disconnect()

    def pause_run(self):
        self.node.sdo[0x6040].bits[8] = 1

    def continue_run(self):
        # self.node.sdo[0x6040].bits[0] = 1
        # self.node.sdo[0x6040].bits[1] = 1
        # self.node.sdo[0x6040].bits[2] = 1
        # self.node.sdo[0x6040].bits[3] = 1
        # self.node.sdo[0x6040].bits[8] = 0
        val = self.node.sdo[0x6040].raw
        val |= 0xF
        val &= 0x7F
        self.node.sdo[0x6040].raw = val

    def _decTohex(self,number):
        """decimal to hexadecimal"""
        if (number < 0):
            number = hex(number & 0xffff)  # negetive decimal number
        else:
            number = hex(number)  # positive decimal number
        # print number
        if (len(number) == 3):
            number = '0' + number[2] + '00'
        elif (len(number) == 4):
            number = number[2:4] + '00'
        elif (len(number) == 5):
            number = number[3:5] + '0' + number[2]
        elif (len(number) == 6):
            number = number[4:6] + number[2:4]
        number = number.decode('hex')
        return number

    def _decTohex_32(self,number):
        """decimal to hexadecimal"""
        if (number < 0):
            number = hex(number & 0xffffffff)  # negetive decimal number
            number = number[8:10] + number[6:8] + number[4:6] + number[2:4]
            return number.decode('hex')

        else:
            number = hex(number)  # positive decimal number
            # print number
            if (len(number) == 10):
                number = number[8:10] + number[6:8] + number[4:6] + number[2:4]
            elif (len(number) == 9):
                number = number[7:9] + number[5:7] + number[3:5] + '0' + number[2]
            elif (len(number) == 8):
                number = number[6:8] + number[4:6] + number[2:4] + '00'
            elif (len(number) == 7):
                number = number[5:7] + number[3:5] + '0' + number[2] + '00'
            elif (len(number) == 3):
                number = '0' + number[2] + '000000'
            elif (len(number) == 4):
                number = number[2:4] + '000000'
            elif (len(number) == 5):
                number = number[3:5] + '0' + number[2] + '0000'
            elif (len(number) == 6):
                number = number[4:6] + number[2:4] + '0000'

            return number.decode('hex')

    def _I100_msg_to_device(self, position):
        """
        rad -> mdeg * resolution of encoder * reduction ratio / 360 degree
        :return:
        """
        position = degrees(position) * 4096 * 457 / 360
        return int(position)

    def _I100_msg_from_device(self, position):
        """
        actual position [count] -> rad
        :return:
        """
        position = radians(position) * 360 / 457 / 4096
        return position

    def _eds_configure(self):
        """
        :return:
        """
        # self.node.sdo[0x1006].raw = 1  # communication cycle period ms
        # self.node.sdo[0x100c].raw = 100  # node  guard time ms
        # self.node.sdo[0x100d].raw = 3  # life time factor
        # self.node.sdo[0x1014].raw = 163  # emergency object id
        # self.node.sdo[0x1003][0x0].raw = 0  # number of error
        # self.node.sdo[0x6060].raw = 1  # torque mdoe
        # self.node.sdo[0x6086].raw = 0  # motion profile type

        # #self.node.sdo[0x6510][3].raw = 600  # Amplifier peak current limit
        # #self.node.sdo[0x6510][4].raw = 300  # Amplifier continuous current limit
        # #self.node.sdo[0x6510][5].raw = 1000  # Amplifer time at peak current
        # #self.node.sdo[0x6510][6].raw = 560  # Amplifier maximum voltage

        # self.node.sdo[0x2110].raw = 600  # Peak current limit
        # self.node.sdo[0x2111].raw = 164  # Continuous current limit
        # self.node.sdo[0x2112].raw = 1234  # Time at peak current limit

        # #profile position mode
        # self.node.sdo[0x60fb][0x1].raw = 1000 # Pp - Position loop proportional
        # self.node.sdo[0x60fb][0x2].raw = 16384 # Vff - Velocity feed forward
        # self.node.sdo[0x60fb][0x3].raw = 0 # Aff - Acceleration feed forward
        # self.node.sdo[0x2121].raw = 1638400 # Trajectory jerk limit
        # self.node.sdo[0x6083].raw = 409600 # Profile Acceleration
        # self.node.sdo[0x6084].raw = 409600 # Profile Dcceleration
        # self.node.sdo[0x6085].raw = 409600 # Quick Stop Deceleration

        # #profile velocity mode
        # self.node.sdo[0x60f9][0x1].raw = 1401  # Proportional gain for velocity loop
        # self.node.sdo[0x60f9][0x2].raw = 442  # Integral gain for velocity loop

        # #profilee torque mode
        # self.node.sdo[0x60f6][0x1].raw = 48  # Proportional gain for current loop
        # self.node.sdo[0x60f6][0x2].raw = 21  # Integral gain for current loop


        # #motor setting
        # #pdb.set_trace()
        # '''
        # self.node.sdo[0x6410][0x1].raw = 0  # motor type
        # self.node.sdo[0x6410][0x2].raw = 1  # motor pole pairs
        # self.node.sdo[0x6410][0x3].raw = 0  # Motor wiring reversed
        # self.node.sdo[0x6410][0x4].raw = 0  # motor hall type
        # self.node.sdo[0x6410][0x5].raw = 0  # Motor hall wiring
        # self.node.sdo[0x6410][0x6].raw = 0  # motor hall offset
        # self.node.sdo[0x6410][0x7].raw = 116  # motor resistance
        # self.node.sdo[0x6410][0x8].raw = 33  # motor inductance
        # self.node.sdo[0x6410][0x9].raw = 138000  # motor inertia
        # self.node.sdo[0x6410][0xa].raw = 562  # motor back EMF constant
        # self.node.sdo[0x6410][0xb].raw = 8192000  # motor max velocity
        # self.node.sdo[0x6410][0xc].raw = 5380  # motor torque constant
        # self.node.sdo[0x6410][0xd].raw = 102000  # motor peak torque
        # self.node.sdo[0x6410][0xe].raw = 8820  # motor continuous torque
        # self.node.sdo[0x6410][0xf].raw = 0  # motor has temp sensor
        # self.node.sdo[0x6410][0x10].raw = 1  # motor has brake
        # self.node.sdo[0x6410][0x11].raw = 0  # Delay from error to brake active
        # self.node.sdo[0x6410][0x12].raw = 0  # Delay from brake to PWM disable
        # self.node.sdo[0x6410][0x13].raw = 0  # Velocity where brake may be activated without delay
        # self.node.sdo[0x6410][0x14].raw = 6  # Motor encoder type
        # self.node.sdo[0x6410][0x15].raw = 0  # Motor encoder unit
        # self.node.sdo[0x6410][0x16].raw = 0  # Encoder direction
        # self.node.sdo[0x6410][0x17].raw = 4096  # Encoder counts per rev
        # self.node.sdo[0x6410][0x18].raw = 100  # Encoder resolution
        # self.node.sdo[0x6410][0x19].raw = 100000  # Encoder electrical distance (linear only)
        # self.node.sdo[0x6410][0x1a].raw = 0  # Encoder index pulse distance
        # self.node.sdo[0x6410][0x1b].raw = 0  # Motor Units
        # self.node.sdo[0x6410][0x1c].raw = 0  # Shift amount for analog encoder
        # self.node.sdo[0x6410][0x1d].raw = 4000  # position encoder resolution
        # self.node.sdo[0x6410][0x1e].raw = 0  # position encoder type
        # self.node.sdo[0x6410][0x1f].raw = 0  # position encoder direction
        # '''
        self.node.sdo[0x6040].bits[0] = 1
        self.node.sdo[0x6040].bits[1] = 1
        self.node.sdo[0x6040].bits[2] = 1
        self.node.sdo[0x6040].bits[3] = 1
        self.node.sdo[0x6040].bits[8] = 0
        self.node.sdo[0x605d].raw = 1 # halt options
