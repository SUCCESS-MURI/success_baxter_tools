#!/usr/bin/python

import rospy
import baxter_interface
from success_baxter_tools.motion import solve_IK
from geometry_msgs.msg import(
    Pose
)

def main():
    rospy.init_node('iksolver_test')

    _limb = baxter_interface.Limb('left')

    target_pose = Pose()
    target_pose.orientation.y = 1
    target_pose.position.x = 0.6
    target_pose.position.y = 0.4

    joint_state = solve_IK('left', target_pose)
    joint_angles = dict(zip(joint_state.name,joint_state.position))
    _limb.move_to_joint_positions(joint_angles)


    target_pose.position.x = 0.8
    target_pose.position.z = 0.2

    joint_state = solve_IK('left', target_pose)
    joint_angles = dict(zip(joint_state.name,joint_state.position))
    _limb.move_to_joint_positions(joint_angles)

if __name__ == '__main__':
    main()
