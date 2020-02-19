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
theta = 0


def poseCallBack(pose_message):
    global x, y, theta

    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta


def move(linear_speed, desired_distance, isForward):
    global x
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

    t0 = rospy.Time.now().to_sec()
    loop_rate = rospy.Rate(100)

    while True:
        rospy.loginfo("Turtlesim moving forward")
        vel_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()

        traveled_distance = linear_speed * (t1 - t0)
        rospy.spin
        loop_rate.sleep()

        print traveled_distance
        if (traveled_distance > desired_distance):
            rospy.loginfo("Reached")
            break

    vel_msg.linear.x = 0
    vel_publisher.publish(vel_msg)


def rotate(angular_speed_rad, desired_angle, isClockwise):
    global theta
    vel_msg = Twist()  # creating an instance of Twist message type

    if (isClockwise):
        vel_msg.angular.z = abs(angular_speed_rad)
    else:
        vel_msg.angular.z = -abs(angular_speed_rad)

    # initializing parameters
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    traveled_angle = 0.0

    t0 = rospy.Time.now().to_sec()
    loop_rate = rospy.Rate(100)

    while True:
        rospy.loginfo("Turtlesim rotating")
        vel_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()

        traveled_angle = angular_speed_rad * (t1 - t0)
        rospy.spin
        loop_rate.sleep()

        print radiansToDegree(traveled_angle)
        if (radiansToDegree(traveled_angle) > desired_angle):
            rospy.loginfo("Reached")
            break

    vel_msg.angular.z = 0
    vel_publisher.publish(vel_msg)


def moveToGoal(goal_pose_x, goal_pose_y, distance_tolerance):
    global x, y, theta
    vel_msg = Twist()  # creating an instance of Twist message type
    loop_rate = rospy.Rate(100)
    E = 0
    kp_linear = 0.5
    kp_angular = 4

    while True:
        distance = math.sqrt((x-goal_pose_x)**2 + (y-goal_pose_y)**2)
        vel_msg.linear.x = kp_linear * distance

        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        angle = math.atan2(goal_pose_y - y, goal_pose_x - x)
        vel_msg.angular.z = kp_angular * (angle - theta)

        vel_publisher.publish(vel_msg)

        print "distance: {}, x: {}, y: {}".format(distance, x, y)
        if (distance < distance_tolerance):
            print("reached goal")
            break
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    vel_publisher.publish(vel_msg)


def degreesToRadians(angle_in_degrees):
    return angle_in_degrees * math.pi / 180


def radiansToDegree(angle_in_radians):
    return angle_in_radians * 180 / math.pi


def setDesiredOrientation(angle_rad_desired):
    angle_rad_diff = angle_rad_desired - theta
    if (angle_rad_diff < 0):
        clockwise = True
    else:
        clockwise = False
    rotate(degreesToRadians(50), math.degrees(abs(angle_rad_diff)), clockwise)


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

    # testing move method
    # move(2, 5, False)

    # testing rotate method
    # rotate(1, 180, True)

    # testing movetogoal method
    # moveToGoal(1, 1, 0.5)
    rotate(degreesToRadians(30), 90, True)
    # setDesiredOrientation(degreesToRadians(90))
    Reset()
