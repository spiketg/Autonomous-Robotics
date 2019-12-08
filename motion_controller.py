#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 10 22:00:13 2019

@author: tommy
"""

import rospy
import pf
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from time import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler

error =0.025
error_yaw = 0.075
yaw=pitch=roll=0.0
x=y=0
prev_x=prev_y=prev_yaw=0
x_init = 4.25-0.42
y_init = 2.15-0.35
N=1
previous_parts=np.zeros((N,3))
beams = []
valid_angs = range(0,359,45)
old_state = (0,0,0)


def tf(control):
    x_w = x_init-control[1]
    y_w = y_init - control[0]
    yaw_w = (-90*np.pi/180)-control[2]
    if yaw_w < 0:
        yaw_w = yaw_w%(-2*np.pi)
        if yaw_w < -np.pi:
            yaw_w+=2*np.pi
    else:
        yaw_w = yaw_w = yaw_w%(2*np.pi)
        if yaw_w > np.pi:
            yaw_w-=2*np.pi
    
    return x_w,y_w,yaw_w

def get_rotation(msg):
    
    global prev_x,prev_y,prev_yaw
    global yaw,pitch,roll
    global x,y
    orientation_p = msg.pose.pose.position
    orientation_q = msg.pose.pose.orientation
    pos = [orientation_p.x, orientation_p.y]
    
    x = pos[0]
    y = pos[1]
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    
def get_odom():
    rospy.Subscriber('/odom',Odometry,get_rotation)
    
def laser_scan_callback(data):
    
    global beams 
    beams = []
    for i in valid_angs:
        if data.ranges[i]>0.1:
            
            beams.append(data.ranges[i])
        else:
            beams.append(3.5)
   
def read_laser_scan_data():
    rospy.Subscriber('scan',LaserScan,laser_scan_callback)
 
def move_motor(fwd,ang):
    pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)
    mc = Twist()
    mc.linear.x = fwd
    mc.angular.z = ang
    pub.publish(mc)
    print('lin spd: '+ str(fwd))
    print('ang spd: ' + str(ang))


class Control(object):
    
    def __init__(self, x, v,maxspeed, minspeed, biasAcceleration=0, lin=True, final=False):
        self.lin= lin
        self.final = final 
        if self.lin ==False:
            self.x = yaw
            self.drift = 0.05
        else:
            self.x = x
            self.drift = 0.05
        self.v = v
        self.biasAcceleration = biasAcceleration
        
    def step(self, u, h):
        # Returns the updated position x and velocity v after one time step h
        a = self.biasAcceleration + u 
        #self.x = self.x + h*self.v + 1./2.*a*h**2 # EOM for position
        self.v = self.v + h*a # EOM for velocity
        if abs(self.v) >maxspeed:
            if self.v > 0.:
                self.v = maxspeed
            if self.v < 0.:
                self.v = -maxspeed
        elif abs(self.v) < minspeed:
            if self.v > 0:
                self.v=minspeed
            if self.v < 0:
                self.v = -minspeed
        if self.lin==True:
            move_motor(self.v,0)
        if self.lin==False:
            move_motor(0,self.v)
        
    def sense(self):
        px=prev_x
        py = prev_y
        pyw = prev_yaw
        cont = ((px,py,pyw),(x,y,yaw))
        parts=pf.move_particles(cont,previous_parts,beams)
        state_est = np.mean(parts, axis=0)
        est_x=state_est[0]
        est_y=state_est[1]
        est_yaw = state_est[2]
        if est_yaw > 0:
            est_yaw = est_yaw%(-2*np.pi)
            if est_yaw < -np.pi:
                est_yaw+=2*np.pi
        else:
            est_yaw = est_yaw%(2*np.pi)
            if est_yaw > np.pi:
                est_yaw-=2*np.pi
        if self.lin==True:
            self.x=(est_x,est_y)
            
        else:
            self.x=est_yaw
            
        print 'pf state is: ' +str(state_est)    
        print 'est state is: '+str(x)+', '+str(y)+', '+str(yaw)   
        global previous_parts 
        
        previous_parts= parts
        global prev_x,prev_y,prev_yaw
        prev_x = x
        prev_y = y
        prev_yaw = yaw
        return self.x
    
def move_to_point(system, k_p, k_d, k_i, target):
    sys = system(0, 0)
        
    x_target = target # Set target position
    h = 0.1 # dt
    x_old = sys.sense() # initial position of the system
    e_old = 0.2 # initial error
    e_i = 0 # initial intergral error
    
    reached = False
    
    while not reached:       
        # obtain current position of system  
        rospy.wait_for_message('/odom', Odometry)	
        rospy.wait_for_message('scan',LaserScan)
        s_time = time()
        state=sys.sense()
        curr = time()
        # Implement the discretized PID loop equations
        if sys.lin==False and sys.final==False:
            x_target2 = np.arctan2(x_target[1]-y,x_target[0]-x)
            print 'ang_target = ' +str(x_target2)
            e = x_target2-state
            print 'ang error: '+str(e)#Calculate the error in position 
        if sys.lin==True:
            e = np.sqrt(np.power(target[0]-state[0],2)+np.power(target[1]-state[1],2))
        if sys.lin==False and sys.final ==True:
            e = target[2]-yaw
        e_d = e/h # Calculate derivative of the error
        e_i = e_i+ (e*h) # Calculate integral of the error  
        u = (k_p*e) + (k_d*e_d) + (k_i*e_i)  # Calculate the output of the PID loop
        print 'error is '+str(e)
        if abs(e) < sys.drift:
            reached = True
            move_motor(0,0)
            print 'final error is ' + str(e)
            print('finished')
        else:    
        # apply control effort, u (acceleration)
            sys.step(u, h)
        
        x_old = state # store previous position 
        
        e_old = e # store previous error 
        
    return x_old
        
        # store position, target and time for plotting reasons

# Tune the PID equations
k_p = 0.08
k_d = 0.005
k_i = 0.01

k_p_ang = 0.3
k_i_ang = 0.05
k_d_ang = 0.01

maxspeed = 0.26
minspeed = 0.05

maxang = np.pi/2
minang = -np.pi/2

def move_pose(tar_pose, prev_parts=previous_parts, ostate=old_state):
    global previous_parts, old_state
    old_state = ostate
    previous_parts = prev_parts
    rospy.wait_for_message('/odom', Odometry)	
    rospy.wait_for_message('scan',LaserScan)
    get_odom()
    read_laser_scan_data()
    systemLin = lambda x, v : Control(x,v,maxspeed,minspeed,0, True)
    systemAng1 = lambda yaw, v: Control(yaw,v,maxang,minang,0, False)
    systemAng2 = lambda yaw, v: Control(yaw,v,maxang,minang,0, lin=False, final=True)
    pose = tar_pose 
    move_to_point(systemAng1, k_p_ang, k_d_ang, k_i_ang, pose)
    x_old= move_to_point(systemLin, k_p, k_d, k_i, pose)
    yaw_old = move_to_point(systemAng2, k_p_ang,k_d_ang,k_i_ang,pose)

    xyyaw_old = (x_old[0],x_old[1],yaw_old)
    return previous_parts,xyyaw_old
