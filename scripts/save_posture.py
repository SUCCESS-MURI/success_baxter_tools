#!/usr/bin/python

import rospy
import argparse
import baxter_interface
from success_baxter_tools.motion import save_posture



def main():
    #start the rosnode
    rospy.init_node("save_baxter_posture_node")
    #parse the arguments
    parser = argparse.ArgumentParser(description="Script that saves the current position of Baxter's arm as a list of joint position &" + 
        " enable callback with move_to_posture.py")
    parser.add_argument("posture_name", help="Name of the posture to be called in the future", type=str)
    parser.add_argument("-b","--button", help="If included, will only save the posture after pressing the big spin wheel on either arms", dest="button", action="store_true", default=False)
    parser.add_argument("--record_path", help="Path to the file that store the postures", type=str)
    parser.add_argument("--arm", help="Give the name of the arm if you only want to save the posture of one arm", type=str)
    args = parser.parse_args()
    #parse the arguments

    rospy.loginfo("Saving posture {} ...".format(args.posture_name))
    button_action = args.button
    arm_param = None
    if args.arm:
        arm_param = args.arm
    if args.record_path:
        save_posture(args.posture_name, button_control=button_action, arm=arm_param, record_path=args.record_path)
    else:
        save_posture(args.posture_name, button_control=button_action, arm=arm_param)
    rospy.loginfo("Posture {} save".format(args.posture_name))




if __name__ == "__main__":
    main()    