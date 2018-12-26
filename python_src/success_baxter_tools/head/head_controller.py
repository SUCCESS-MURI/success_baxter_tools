
import rospy

from success_ros_msgs.msg import(
    HeadCmdAction,
    HeadCmdGoal
)
import actionlib

class HeadController(object):

    def __init__(self, topic='head_command_pan'):
        self._client = actionlib.SimpleActionClient(topic, HeadCmdAction)
        self._client.wait_for_server()


    def move_head(self, target, time, wait=True, noise=False):
        """ Move Baxter's head to the target

        parameters
        ----------
        target: float
            In Radian, the position of Baxter's head
        time: float
            In second, time plan for baxter to get to the position
        wait: boolean
            Whether to wait for the function to finish 
        noise: boolean
            Whether there will be idle noise movements
        topic: str
            Topic if different
        """
        goal = HeadCmdGoal
        #goal.enable_pan_request = HeadCmdGoal.REQUEST_PAN_DISABLE
        goal.enable_noise = noise
        goal.target = target
        goal.time = time

        self._client.send_goal(goal)
        if wait:
            self._client.wait_for_result()

