#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty

x = 0
y = 0
z = 0
yaw = 0


def poseCallBack(pose_message):
    global x
    global y, z, yaw

    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta


def move(speed, distance):

    vel_msg = Twist()

    x0 = x
    y0 = y
    distance_moved = 0.0

    vel_msg.linear.x = speed

    loop_rate = rospy.Rate(10)
    vel_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    while True:
        rospy.loginfo("Turtlesim moving forward")
        vel_publisher.publish(vel_msg)
        loop_rate.sleep()

        distance_moved = distance_moved + \
            abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        print distance_moved
        if (distance_moved > distance):
            rospy.loginfo("Reached")
            break

    vel_msg.linear.x = 0
    vel_publisher.publish(vel_msg)


def Reset():
    rospy.wait_for_service('reset')
    reset_turtle = rospy.ServiceProxy('reset', Empty)
    reset_turtle()
    print "Terminating reset"
    rospy.spin()


if __name__ == "__main__":

    rospy.init_node('turtlesim_move_node', anonymous=True)
    pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallBack)
    time.sleep(2)
    move(1, 5)
    Reset()
