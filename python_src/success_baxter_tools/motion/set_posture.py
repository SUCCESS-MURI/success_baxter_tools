#!/usr/bin/env python


"""
Tool that use to set Baxter into different modes
"""

import os
import rospy
import argparse
import baxter_interface
import yaml
from ik_solver import solve_IK
import threading
import alloy.ros

def move_arm_to_pose(limb_name, pose):
    #create the baxter interface 
    limb_interface = baxter_interface.Limb(limb_name)
    #do IK to solve the position
    joint_position = solve_IK(limb_name, pose)
    #zip the name and positions
    joint_position = dict(zip(joint_position.name, joint_position.position))
    #move the limb to the position
    limb_interface.move_to_joint_positions(joint_position)

def move_to_posture(posture_name,record_path="posture_records.yaml", block=True, done_cb=None):

    #rospy.init_node("bax_set_posture")
    left_limb = baxter_interface.Limb('left')
    right_limb = baxter_interface.Limb('right')

    #resolve path
    record_path = alloy.ros.resolve_res_path(record_path,"success_baxter_tools")

    if record_path:
        with open(record_path,'r') as f:
            posture_list = yaml.load(f)
            joint_angles = posture_list[posture_name]
            if 'left' in joint_angles and 'right' in joint_angles:
                lt = threading.Thread(target=left_limb.move_to_joint_positions, args=(joint_angles['left'],))
                rt = threading.Thread(target=right_limb.move_to_joint_positions, args=(joint_angles['right'],))
                lt.start()
                rt.start()
                lt.join()
                rt.join()
            elif 'left' in joint_angles:   
                left_limb.move_to_joint_positions(joint_angles['left'])  
            elif 'right' in joint_angles:
                right_limb.move_to_joint_positions(joint_angles['right'])

def save_posture(posture_name, button_control=True, arm=None, record_path="posture_records.yaml"):

    if button_control:
        left_nav = baxter_interface.Navigator('left')
        right_nav = baxter_interface.Navigator('right')

        while not left_nav.button0 and not right_nav.button0:
            rospy.sleep(0.1)
    
    #save the position
    left_joint_angles = baxter_interface.Limb('left').joint_angles()
    right_joint_angles = baxter_interface.Limb('right').joint_angles()

    posture_list = dict()
    #resolve path
    record_path = alloy.ros.resolve_res_path(record_path,"success_baxter_tools")
    #create the file at the root of `success_baxter_tools\res` if doesn't exist
    if record_path is None:
        record_path = os.path.join(alloy.ros.create_res_dir("success_baxter_tools"),"posture_records.yaml")

    #save them to some type of files
    if not os.path.exists(record_path):
        yaml.dump(posture_list,file(record_path,'w'))

    with open(record_path,'rw') as f:
        posture_list = yaml.load(f)
        if arm == 'right':
            posture_list[posture_name] = {
                'right': right_joint_angles
            }
        elif arm == 'left':
            posture_list[posture_name] = {
                'left': left_joint_angles,
            }
        else:
            posture_list[posture_name] = {
                'left': left_joint_angles,
                'right': right_joint_angles
            }

    yaml.dump(posture_list, file(record_path,'w'))