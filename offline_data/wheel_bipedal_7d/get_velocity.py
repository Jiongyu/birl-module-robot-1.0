#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys
from math import radians,degrees
import string

sys.path.append("~/Desktop")

joint_pos = []
joint_vel = []

# rad
max_vel = radians(10) 


def load_data(text):
    
    data = text.split('\n')
    joint_pos.append([0,0,0,0,0,0,0])
    for i in range(len(data)):

        temp1 = data[i].replace(',','')    #删除逗号
        temp2 = temp1.replace(';','')   #删除分号
        data[i] = temp2.replace('=',' ')  #用空格替代等号
        # print data[i]
        if data[i].startswith('P'):      
            s = data[i].split()    #字符串按照空格分开，split()默认以空格分开 
            s[1] = radians(string.atof(s[1]))    #字符串转化为浮点数
            s[2] = radians(string.atof(s[2]))    #字符串转浮点数
            s[3] = radians(string.atof(s[3]))
            s[4] = radians(string.atof(s[4]))
            s[5] = radians(string.atof(s[5]))
            s[6] = radians(string.atof(s[6]))
            s[7] = radians(string.atof(s[7]))
            joint_pos.append(s[1:8])

def get_velocity(pos):

    for i in range(len(pos) - 1):

        temp_vel = []
        temp_max_pos = 0
        temp_time = 0

        for j in range(len(pos[i])):

            temp = pos[i+1][j] - pos[i][j]

            if abs(temp) > abs(temp_max_pos):
                temp_max_pos = temp
        
        if temp_max_pos > 0:
            temp_time = temp_max_pos / max_vel
        elif temp_max_pos < 0:
            temp_time = temp_max_pos / (-max_vel)
        else:
            temp_time = 0
        # print degrees( temp_max_pos )
        # print temp_time

        for j in range(len(pos[i])):
            
            temp_vel.append((pos[i+1][j] - pos[i][j])/temp_time)

        joint_vel.append(temp_vel)

def save_file():

    with open ("pos_and_vel.txt","w") as f:
        for i in range(len(joint_pos)-1):

            f.write("P" + str(i) + "=")
            for j in range(len(joint_pos[i])):
                f.write(str(degrees(joint_pos[i+1][j])) + ' ' + ',')
            f.write(';\n')

            f.write("V" + str(i) + "=")
            for j in range(len(joint_vel[i])):
                f.write(str(round(degrees(joint_vel[i][j]),3)) + ' ' + ',')
            f.write(';\n\n')

def main():
    with open("position_Data.txt","r") as f:
        text = f.read()
        # print text
        load_data(text)
        # print joint_pos
        get_velocity(joint_pos) 
        pass
        
        # for i in range(len(joint_pos)-1):
        #     print "\npos&vel"
        #     print joint_pos[i+1]
        #     print joint_vel[i]

        save_file()


if __name__ == "__main__":
    main()