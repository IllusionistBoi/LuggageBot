import rospy
from time import time, sleep
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from random import randrange


SPEED = 0.001

class Animate():
    def __init__(self):
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)


    def _go(self, up, down, left, right):
        rec = self.get_state("green_cylinder", "link")
        req = ModelState()
        req.model_name = "green_cylinder"
        req.twist.linear.x = 0.0
        req.twist.linear.y = 0.0
        req.twist.linear.z = 0.0
        req.twist.angular.x = 0.0
        req.twist.angular.y = 0.0
        req.twist.angular.z = 0.0
        req.pose.position.x = rec.pose.position.x
        req.pose.position.y = rec.pose.position.y
        if up:
            req.pose.position.x = rec.pose.position.x + SPEED
        if down:
            req.pose.position.x = rec.pose.position.x - SPEED
        if left:
            req.pose.position.y = rec.pose.position.y + SPEED
        if right:
            req.pose.position.y = rec.pose.position.y - SPEED
        req.pose.position.z = rec.pose.position.z
        req.pose.orientation.x = rec.pose.orientation.x
        req.pose.orientation.y = rec.pose.orientation.y
        req.pose.orientation.z = rec.pose.orientation.z
        req.pose.orientation.w = 1.0
        self.set_state(req)

    def move(self, up=False, down=False, left=False, right=False, time_in_secs=6):
        start_time = time()
        while time() - start_time < time_in_secs:
            self._go(up, down, left, right)
        sleep(1)

    def run(self):
        # self.move(left=True)
        # self.move(up=True, time_in_secs=12)
        # self.move(left=True)
        # self.move(up=True, time_in_secs=3)
        # self.right(up=True, time_in_secs=12)
        while 1:
            r = randrange(4)
            t = randrange(20)
            if r == 0:
                self.move(left=True, time_in_secs=t)
            if r == 1:
                self.move(right=True, time_in_secs=t)
            if r == 2:
                self.move(up=True, time_in_secs=t)
            if r == 3:
                self.move(down=True, time_in_secs=t)


if __name__ == "__main__":
    a = Animate()
    try:
        a.run()
    except:
        print("Animation failed!")

