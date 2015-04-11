#!/usr/bin/env python

import sys
import rospy, time
import math as math
from nav_msgs.msg import Odometry
from lab4.srv import *
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Point
from tf.transformations import euler_from_quaternion

debug_mode = False
WHEEL_RADIUS = 0.035
DISTANCE_BETWEEN_WHEELS = 0.23

def addAngles(angle1, angle2):
    result = angle1 + angle2

    if result > math.pi:
        result %= math.pi
        result -= math.pi
    elif result < -math.pi:
        result %= -math.pi
        result += math.pi

    if debug_mode:
        print "Angle1: %f, Angle2: %f, Result: %f" % (angle1, angle2, result)

    return result

def publishTwist(x_vel, angular_vel):
    twist = Twist()
    twist.linear.x = x_vel; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angular_vel

    pub.publish(twist)

#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, t):
    x_vel = (WHEEL_RADIUS / 2) * (u1 + u2)
    angular_vel = (WHEEL_RADIUS / DISTANCE_BETWEEN_WHEELS) * (u1 - u2)

    startTime = time.time()

    publishTwist(x_vel, angular_vel)

    while time.time() - startTime < t:
        publishTwist(x_vel, angular_vel)

    publishTwist(0, 0)

pos_tolerance = 0.1
#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    destination_x = current_x + distance * math.cos(current_theta)
    destination_y = current_y + distance * math.sin(current_theta)

    while (current_x > destination_x + pos_tolerance or current_x < destination_x - pos_tolerance) \
            or (current_y > destination_y + pos_tolerance or current_y < destination_y - pos_tolerance):
        publishTwist(speed, 0)

        # #transform the destination, so that it lies on X axis of the robot
        # if original_theta != current_theta:
        #     angle_difference = addAngles(current_theta, -original_theta)
        #     original_theta = current_theta
        #
        #     destination_x = math.cos(angle_difference) * destination_x - math.sin(angle_difference) * destination_y
        #     destination_y = math.sin(angle_difference) * destination_x + math.sin(angle_difference) * destination_y
        #
        #     print "New destination - X: %f, Y: %f; Angle: %f" % (destination_x, destination_y, angle_difference)

        #write logs

        rospy.sleep(rospy.Duration(0, 1))

    publishTwist(0, 0)

angle_tolerance = 0.1
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    angular_vel = 1

    if angle < 0:
        angular_vel = -angular_vel

    destination_angle = addAngles(current_theta, angle)

    if debug_mode:
        print "Current theta: %f, Destination angle: %f, Angle: %f" % (current_theta, destination_angle, angle)

    while current_theta > destination_angle + angle_tolerance or current_theta < destination_angle - angle_tolerance:
        publishTwist(0, angular_vel)

        #write logs

        rospy.sleep(rospy.Duration(0, 1))

    publishTwist(0, 0)

#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    angular_vel = speed / radius

    if angle < 0:
        angular_vel = -angular_vel

    destination_angle = addAngles(current_theta, angle)

    while current_theta > destination_angle + angle_tolerance or current_theta < destination_angle - angle_tolerance:
        publishTwist(speed, angular_vel)

        rospy.sleep(rospy.Duration(0, 1))

    publishTwist(0, 0)

#Odometry Callback function.
def read_odometry(msg):
    orientation_quat = msg.pose.pose.orientation
    q = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]

    roll, pitch, yaw = euler_from_quaternion(q)

    global current_x
    global current_y
    global current_theta
    global receivedInitialPos

    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    current_theta = yaw

    if not receivedInitialPos:
        receivedInitialPos = True

#Callback function that processes the initial position received.
def processGoalPos(goalPos):
    global goalPosX
    global goalPosY
    global receivedGoalPos

    goalPosX = goalPos.pose.position.x
    goalPosY = goalPos.pose.position.y

    receivedGoalPos = True

def sendRequest(initPos, goalPos):
    print "Entered sendRequest()..."

    rospy.wait_for_service('calculateTrajectory')

    print "Finished wait_for_service..."

    try:
        calculateTraj = rospy.ServiceProxy('calculateTrajectory', Trajectory)
        trajectory = calculateTraj(initPos, goalPos)

        return trajectory
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

#This function sequentially calls methods to perform a trajectory.
def executeTrajectory(trajectory):
    waypoint_counter = 0

    for point in trajectory.path.poses:
        goalX = point.pose.position.x
        goalY = point.pose.position.y

        waypoint_counter = waypoint_counter + 1
        print "Going to Waypoint %d" % waypoint_counter
        goToPosition(goalX, goalY)

def goToPosition(goalX, goalY):
    global current_x
    global current_y
    global current_theta

    xDiff = goalX - current_x
    yDiff = goalY - current_y

    # adding current_theta is done in rotate(angle)
    angle = math.atan2(yDiff, xDiff) - current_theta
    print "Rotating by angle %f" % angle
    rotate(angle)

    distance = math.sqrt(pow(xDiff, 2) + pow(yDiff, 2))
    print "Driving forward by distance: %f" % distance
    driveStraight(.3, distance)

if __name__ == "__main__":
    rospy.init_node('lab4_path_control')

    global receivedInitialPos
    global receivedGoalPos
    receivedInitialPos = False
    receivedGoalPos = False

    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5) # Publisher for commanding robot motion
    odom_sub = rospy.Subscriber('/odom', Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages
    goal_pos_sub = rospy.Subscriber('/goalPos', PoseStamped, processGoalPos, queue_size=1)

    print "Waiting for the initial and goal positions..."
    while not receivedInitialPos or not receivedGoalPos:
        pass
    print "Received initial and goal positions!"

    print "Initial Position: X(%f), Y(%f)" % (current_x, current_y)
    print "Goal Position: X(%f), Y(%f)" % (goalPosX, goalPosY)

    # doesn't matter what the values are
    initPos = PoseWithCovarianceStamped()
    initPos.pose.pose.position.x = current_x
    initPos.pose.pose.position.y = current_y
    goalPos = PoseStamped()
    goalPos.pose.position.x = goalPosX
    goalPos.pose.position.y = goalPosY

    print "About to send request to path_planner to get trajectory..."

    #  give initial and goal to find all waypoints; when handleRequest is called
    trajectory = sendRequest(initPos, goalPos)

    print "Starting trajectory"

    executeTrajectory(trajectory)

    print "Finished trajectory!"




