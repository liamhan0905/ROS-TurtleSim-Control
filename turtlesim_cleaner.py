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


def move(linear_speed, desired_distance, isForward):

    vel_msg = Twist()  # creating an instance of Twist message type

    if (isForward):
        vel_msg.linear.x = abs(linear_speed)
    else:
        vel_msg.linear.x = -abs(linear_speed)

    # initializing parameters
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    traveled_distance = 0.0

    t0 = rospy.get_time()
    loop_rate = rospy.Rate(100)

    while True:
        rospy.loginfo("Turtlesim moving forward")
        vel_publisher.publish(vel_msg)
        t1 = rospy.get_time()

        traveled_distance = linear_speed * (t1 - t0)
        rospy.spin
        loop_rate.sleep()

        print traveled_distance
        if (traveled_distance > desired_distance):
            rospy.loginfo("Reached")
            break

    vel_msg.linear.x = 0
    vel_publisher.publish(vel_msg)


def rotate(angular_speed, desired_angle, isClockwise):
    vel_msg = Twist()  # creating an instance of Twist message type

    if (isClockwise):
        vel_msg.angular.z = abs(angular_speed)
    else:
        vel_msg.angular.z = -abs(angular_speed)

    # initializing parameters
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    traveled_angle = 0.0

    t0 = rospy.get_time()
    loop_rate = rospy.Rate(100)

    while True:
        rospy.loginfo("Turtlesim rotating")
        vel_publisher.publish(vel_msg)
        t1 = rospy.get_time()

        traveled_angle = angular_speed * (t1 - t0)
        rospy.spin
        loop_rate.sleep()

        print traveled_angle
        if (traveled_angle > desired_angle):
            rospy.loginfo("Reached")
            break

    vel_msg.angular.z = 0
    vel_publisher.publish(vel_msg)


def moveToGoal():
    print movetot = goals
    pass


def degreesToRadians(angle_in_degrees):
    return angle_in_degrees * math.pi / 180


def Reset():
    rospy.wait_for_service('reset')
    reset_turtle = rospy.ServiceProxy('reset', Empty)
    reset_turtle()
    print "Terminating reset"
    rospy.spin()


if __name__ == "__main__":

    rospy.init_node('turtlesim_move_node', anonymous=True)
    vel_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallBack)
    time.sleep(2)
    move(2, 5, False)
    rotate(1, 180, True)
    Reset()
