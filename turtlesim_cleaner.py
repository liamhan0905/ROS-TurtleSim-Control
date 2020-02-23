#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty

# initialize x,y,theta
x = 0
y = 0
theta = 0


def poseCallBack(pose_message):
    global x, y, theta

    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta


def move(linear_speed, desired_distance, isForward):
    global x, y
    # set current position as initial position
    x0 = x
    y0 = y
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

    # t0 = rospy.Time.now().to_sec()
    loop_rate = rospy.Rate(100)

    while True:
        # rospy.loginfo("Turtlesim moving forward")
        vel_publisher.publish(vel_msg)
        # t1 = rospy.Time.now().to_sec()

        loop_rate.sleep()

        traveled_distance = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))

        # traveled_distance = linear_speed * (t1 - t0)

        print "distance: {}, x: {}, y: {}, theta: {}".format(traveled_distance, x, y, theta)
        if (traveled_distance > desired_distance):
            rospy.loginfo("Reached")
            break

    vel_msg.linear.x = 0
    vel_publisher.publish(vel_msg)


def rotate(angular_speed_rad, desired_angle, isClockwise):
    global theta
    vel_msg = Twist()  # creating an instance of Twist message type

    if (isClockwise):
        vel_msg.angular.z = -abs(angular_speed_rad)
    else:
        vel_msg.angular.z = abs(angular_speed_rad)

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

        loop_rate.sleep()

        traveled_angle = angular_speed_rad * (t1 - t0)

        print radiansToDegree(traveled_angle)
        if (traveled_angle > degreesToRadians(desired_angle)):
            rospy.loginfo("Reached")
            break

    vel_msg.angular.z = 0
    vel_publisher.publish(vel_msg)


def moveToGoal(goal_pose, distance_tolerance):
    global x, y, theta

    vel_msg = Twist()  # creating an instance of Twist message type
    loop_rate = rospy.Rate(100)
    E = 0
    kp_linear = 0.5
    kp_angular = 4

    while True:
        distance = math.sqrt((x-goal_pose.x)**2 + (y-goal_pose.y)**2)
        vel_msg.linear.x = kp_linear * distance

        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        angle = math.atan2(goal_pose.y - y, goal_pose.x - x)
        vel_msg.angular.z = kp_angular * (angle - theta)

        vel_publisher.publish(vel_msg)

        print "distance: {}, x: {}, y: {}, theta: {}".format(distance, x, y, theta)
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


def setDesiredOrientation(angle_deg_desired):
    angle_rad_desired = degreesToRadians(angle_deg_desired)
    angle_rad_diff = angle_rad_desired - theta
    print angle_rad_diff
    if (angle_rad_diff < 0):
        clockwise = True
    else:
        clockwise = False
    rotate(degreesToRadians(50), radiansToDegree(angle_rad_diff), clockwise)


def Reset():
    rospy.wait_for_service('reset')
    reset_turtle = rospy.ServiceProxy('reset', Empty)
    reset_turtle()
    print "Terminating reset"
    rospy.spin()


def gridClean():
    pose = Pose()
    pose.x = 1
    pose.y = 1
    pose.theta = 0
    moveToGoal(pose, 1)
    setDesiredOrientation(degreesToRadians(pose.theta))

    move(2.0, 9.0, True)
    rotate(degreesToRadians(20), 90, False)
    move(2.0, 9.0, True)
    rotate(degreesToRadians(20), 90, False)
    move(2.0, 1.0, True)
    rotate(degreesToRadians(20), 90, False)
    move(2.0, 9.0, True)
    rotate(degreesToRadians(20), 90, True)
    move(2.0, 1.0, True)
    rotate(degreesToRadians(20), 90, True)
    move(2.0, 9.0, True)
    pass


def spiralClean():
    global x, y
    vel_msg = Twist()
    loop_rate = rospy.Rate(1)
    wk = 4
    rk = 0

    while((x < 10.5) and (y < 10.5)):
        rk = rk+1
        vel_msg.linear.x = rk
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = wk
        vel_publisher.publish(vel_msg)
        loop_rate.sleep()

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    vel_publisher.publish(vel_msg)


if __name__ == "__main__":
    try:

        rospy.init_node('turtlesim_move_node', anonymous=True)
        vel_publisher = rospy.Publisher(
            '/turtle1/cmd_vel', Twist, queue_size=10)
        pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallBack)
        time.sleep(2)

        # testing move method going forward
        # move(1.0, 5, True)

        # testing move method going backward
        # move(1.0, 5, False)

        # testing rotate method going clockwise
        # rotate(1.0, 90, True)

        # testing rotate method going counter clockwise
        # rotate(1.0, 90, False)

        # testing goToGoal method
        # desired_coord = Pose()
        # desired_coord.x = 1
        # desired_coord.y = 1
        # moveToGoal(desired_coord, 0.01)

        # testing gridClean method
        # gridClean()

        # testing spiralClean method
        spiralClean()

        # Reset()

    except rospy.ROSInterruptException:
        rospy.loginfo("Error: Node terminated")
