from success_baxter_tools.head import HeadController


import rospy

if __name__ == "__main__":
    rospy.init_node('move_head_demo')
    hm = HeadController()
    hm.move_head(1,10,wait=True)
    rospy.sleep(1)
    hm.move_head(-1,1,wait=True)
    rospy.sleep(1)
    hm.move_head(0,0,wait=True)
    rospy.sleep(1)
    hm.move_head(1,5,wait=False)
    rospy.sleep(1)
    hm.move_head(-1,1,wait=True)