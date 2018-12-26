#!/usr/bin/env python

import rospy

from baxter_core_msgs.msg import(
    HeadPanCommand,
    HeadState
)
import actionlib
from success_ros_msgs.msg import(
    HeadCmdAction,
    HeadCmdResult
)

import numpy as np
import baxter_interface
import threading
       
class HeadController:
    def __init__(self):
        self._curr_pan_state = 0
        self._curr_pan_state_lock = threading.RLock()
        self._target = 0
        self._enable_noise = False
        self._done = True

        #controller
        self.pub = rospy.Publisher('/robot/head/command_head_pan', HeadPanCommand, queue_size=2)
        self._head_sub = rospy.Subscriber('/robot/head/head_state', HeadState, self._head_state_cb, queue_size=1)
        self._head_server = actionlib.SimpleActionServer("head_command_pan", HeadCmdAction, self._head_cb, auto_start=False)
        self._head_server.start()
        #self._head_interface = baxter_interface.head.Head()

        self._noise_rate = rospy.Rate(10)
        
    def _head_state_cb(self, msg):
        with self._curr_pan_state_lock:
            self._curr_pan_state = msg.pan
            #check if we are close to target
            if not self._done and np.isclose(msg.pan, self._target, atol=0.05):
                self._done = True

    def _head_cb(self, goal):
        self._enable_noise = goal.enable_noise
        self._goal_time = goal.time
        with self._curr_pan_state_lock:  
            self._target = goal.target
            self._done = False

        #now we wait for the execution to be done
        while not self._done and not rospy.is_shutdown() and not self._head_server.is_preempt_requested():
            rospy.sleep(0.1)
        #return
        result = HeadCmdResult()
        result.complete = True
        self._head_server.set_succeeded(result)        

    def spin(self):

        while not rospy.is_shutdown():

            msg = HeadPanCommand()
            msg.enable_pan_request = msg.REQUEST_PAN_DISABLE

            #move the head
            if not self._done:
                #set the target
                msg.target = self._target
                #speed ratio will be a function og goal time
                msg.speed_ratio = np.clip(1/( 2 * (self._goal_time + 1e-8)), 0.01,1)
            elif self._enable_noise:
                #add noise
                #TODO add perlin noise
                noise = 0
                msg.target = self._target + noise

            self.pub.publish(msg)
            self._noise_rate.sleep()
                
if __name__ == '__main__':
    rospy.init_node('head_controller')
    head = HeadController()
    rospy.loginfo("starting head controller")
    head.spin()
    

