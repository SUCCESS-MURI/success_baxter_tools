#!/usr/bin/python

import rospy
import argparse
import baxter_interface
from success_baxter_tools.camera import CameraController
from sensor_msgs.msg import(
    Image 
)
import cv2
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


class ImageSaver():

    def _image_cb(self, data):
        try:
            self._last_cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self._show_img:
            cv2.imshow("Image window", self._last_cv_image)
            val = cv2.waitKey(1)
            if val == 120:
                self._quit_flag = True
            elif val == 115:
                self._save_image_to_file(self._last_cv_image)
            # x = 120
            #s = 115
        # try:
        #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # except CvBridgeError as e:
        #   print(e)

    def _save_image_to_file(self, img):
        filename = "{}-{}.png".format(self._file_name_starter, self._img_index)
        cv2.imwrite(filename,img)
        self._img_index += 1

    def __init__(self, show_img=False):
        rospy.Subscriber('image', Image, self._image_cb)
        self._bridge = CvBridge()
        self._last_cv_image = None
        self._show_img = show_img
        self._img_index = 0
        self._file_name_starter = str(rospy.Time.now())
        self._quit_flag = False

def main():
    #start the rosnode
    rospy.init_node("save_images")

    IS = ImageSaver(True)
    while not rospy.is_shutdown():
        if IS._quit_flag:
            break
        rospy.sleep(0.1)

if __name__ == "__main__":
    main()    