#!/usr/bin/python

import rospy
import argparse
import baxter_interface
from success_baxter_tools.camera import CameraController



def main():
    #start the rosnode
    rospy.init_node("save_baxter_posture_node")
    # #parse the arguments
    parser = argparse.ArgumentParser(description="Open camera while making sure other camera is closed")
    parser.add_argument("camera_name", help="Name of camera [left_hand_camera, head_camera, right_hand_camera]", type=str)
    parser.add_argument('-w', "--width", help="Camera image width", default=1280, type=int)
    parser.add_argument('-he', "--height", help="Camera image height", default=800, type=int)
    parser.add_argument('-e', "--exposure", help="Camera exposure level", default=-1, type=int)
    # parser.add_argument("-b","--button", help="If included, will only save the posture after pressing the big spin wheel on either arms", dest="button", action="store_true", default=False)
    # parser.add_argument("--record_path", help="Path to the file that store the postures", type=str)
    # parser.add_argument("--arm", help="Give the name of the arm if you only want to save the posture of one arm", type=str)
    args = parser.parse_args()
    # #parse the arguments

    rospy.loginfo("Starting Camera {} with w:{}, h:{}, exposure:{}".format(args.camera_name, args.width, args.height, args.exposure))

    settings = CameraController.createCameraSettings(width=args.width, height=args.height, exposure=args.exposure) # settings for the first camera
    #settings2 = CameraController.createCameraSettings(width=1280, height=800, exposure=-1) # settings for the second camera
    CameraController.openCameras(args.camera_name, settings=settings)


if __name__ == "__main__":
    main()    