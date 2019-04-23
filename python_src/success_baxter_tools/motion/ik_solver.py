#!/usr/bin/python

import rospy
from geometry_msgs.msg import (
    Pose,
    PoseStamped
)
_trac_imported_flag = True
try:
    from trac_ik_baxter.srv import(
        GetConstrainedPositionIK,
        GetConstrainedPositionIKRequest    
    )
except ImportError:
    print('Unable to import trac_IK, default to Baxter IK')
    _trac_imported_flag = False 

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

from baxter_pykdl import baxter_kinematics

def solve_IK_PseudoInverse(arm, pose):
    """
    Given a pose, returns the IK solution to the desire pose

    parameters
    ----------
    arm: str
        Which arm you try to solve the pose for
    pose: geometry_msgs/Pose or geometry_msgs/PoseStamped
        The desire pose. If no header information is given, the current time and 'base' frame_id will be used 

    returns
    -------
    jnt: sensor_msgs/JointState
        The result of the IK
    """

    #if PoseStamped change to Pose
    if hasattr(pose, 'header'):
        pose = pose.pose
    
    kin = baxter_kinematics(arm)

    #get pseudo inverse
    kin.jacobian_pseudo_inverse()


    print '\n*** Baxter Description ***\n'
    kin.print_robot_description()
    print '\n*** Baxter KDL Chain ***\n'
    kin.print_kdl_chain()
    # FK Position
    print '\n*** Baxter Position FK ***\n'
    print kin.forward_position_kinematics()
    # FK Velocity
    # print '\n*** Baxter Velocity FK ***\n'
    # kin.forward_velocity_kinematics()
    # IK
    print '\n*** Baxter Position IK ***\n'
    pos = [0.582583, -0.180819, 0.216003]
    rot = [0.03085, 0.9945, 0.0561, 0.0829]
    print kin.inverse_kinematics(pos)  # position, don't care orientation
    print '\n*** Baxter Pose IK ***\n'
    print kin.inverse_kinematics(pos, rot)  # position & orientation
    # Jacobian
    print '\n*** Baxter Jacobian ***\n'
    print kin.jacobian()
    # Jacobian Transpose
    print '\n*** Baxter Jacobian Tranpose***\n'
    print kin.jacobian_transpose()
    # Jacobian Pseudo-Inverse (Moore-Penrose)
    print '\n*** Baxter Jacobian Pseudo-Inverse (Moore-Penrose)***\n'
    print 


    #change Pose to PoseStamped
    if not hasattr(pose,'header'):
        tmp = PoseStamped()
        tmp.header.frame_id = 'base'
        tmp.header.stamp = rospy.Time.now()
        tmp.pose = pose
        pose = tmp

    #if trac exist
    if _trac_imported_flag:
        service_addr = 'trac_ik_' + arm
        service_proxy = rospy.ServiceProxy(service_addr, GetConstrainedPositionIK)
        request = GetConstrainedPositionIKRequest()
        request.pose_stamp = [pose]
        request.end_tolerance = 1e-4
        request.num_steps = 10
        response = service_proxy(request)
        if response.isValid[0]:
            return response.joints[0]
        else:
            return None
    else:
        service_proxy = rospy.ServiceProxy('/ExternalTools/' + arm + '/PositionKinematicsNode/IKService', SolvePositionIK)
        request = SolvePositionIKRequest()
        request.pose_stamp.append(pose)
        response = service_proxy(request)
        if response.isValid[0]:
            return response.joints[0]
        return None        


def solve_IK(arm, pose):
    """
    Given a pose, returns the IK solution to the desire pose

    parameters
    ----------
    arm: str
        Which arm you try to solve the pose for
    pose: geometry_msgs/Pose or geometry_msgs/PoseStamped
        The desire pose. If no header information is given, the current time and 'base' frame_id will be used 

    returns
    -------
    jnt: sensor_msgs/JointState
        The result of the IK
    """
    #change Pose to PoseStamped
    if not hasattr(pose,'header'):
        tmp = PoseStamped()
        tmp.header.frame_id = 'base'
        tmp.header.stamp = rospy.Time.now()
        tmp.pose = pose
        pose = tmp

    #if trac exist
    if _trac_imported_flag:
        service_addr = 'trac_ik_' + arm
        service_proxy = rospy.ServiceProxy(service_addr, GetConstrainedPositionIK)
        request = GetConstrainedPositionIKRequest()
        request.pose_stamp = [pose]
        request.end_tolerance = 1e-4
        request.num_steps = 10
        response = service_proxy(request)
        if response.isValid[0]:
            return response.joints[0]
        else:
            return None
    else:
        service_proxy = rospy.ServiceProxy('/ExternalTools/' + arm + '/PositionKinematicsNode/IKService', SolvePositionIK)
        request = SolvePositionIKRequest()
        request.pose_stamp.append(pose)
        response = service_proxy(request)
        if response.isValid[0]:
            return response.joints[0]
        return None        
