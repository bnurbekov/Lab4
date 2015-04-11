#!/usr/bin/env python

import sys
import rospy, time
import math as math
from nav_msgs.msg import Odometry
from lab4.srv import *
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Point
from tf.transformations import euler_from_quaternion
from kobuki_msgs.msg import BumperEvent

debug_mode = False

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
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)

    twist = Twist()
    twist.linear.x = x_vel; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angular_vel

    pub.publish(twist)

WHEEL_RADIUS = 0.035
DISTANCE_BETWEEN_WHEELS = 0.23

#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, t):
    x_vel = (WHEEL_RADIUS / 2) * (u1 + u2)
    angular_vel = (WHEEL_RADIUS / DISTANCE_BETWEEN_WHEELS) * (u1 - u2)

    startTime = time.time()

    publishTwist(x_vel, angular_vel)

    while time.time() - startTime < t:
        publishTwist(x_vel, angular_vel)

    publishTwist(0, 0)

#This function returns the list of speeds for acceleration/deceleration
def createSpeedList(speed, numberOfFractions):
    resultArray = []
    diff = float(speed)/numberOfFractions

    for i in range(0, numberOfFractions):
        resultArray.append(diff*i)

    return resultArray

pos_tolerance = 0.1
#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    currentIndex = 0
    speedList = createSpeedList(speed, 10)
    isDriving = True

    destination_x = current_x + distance * math.cos(current_theta)
    destination_y = current_y + distance * math.sin(current_theta)

    while isDriving:
        # print "x: %f, y: %f, dest x: %f, dest y: %f" %(current_x,current_y, destination_x, destination_y)

        isAccelerating = (current_x > destination_x + pos_tolerance or current_x < destination_x - pos_tolerance) \
            or (current_y > destination_y + pos_tolerance or current_y < destination_y - pos_tolerance)

        publishTwist(speedList[currentIndex], 0)

        if isAccelerating:
            if currentIndex < len(speedList) - 2:
                if debug_mode:
                    print "Accelerating: %f" % (speedList[currentIndex])
                currentIndex += 1
            else:
                if debug_mode:
                    print "Constant speed: %f" % (speedList[currentIndex])
        else:
            if currentIndex >= 0:
                if debug_mode:
                    print "Decelerating: %f" % (speedList[currentIndex])
                currentIndex -= 1
            else:
                isDriving = False

        #write logs

        rospy.sleep(rospy.Duration(0, 1))

#Accepts an angle and makes the robot rotate around it.
# def rotate(angle):
#     angular_vel = 1
#     currentIndex = 0
#     angularVelList = createSpeedList(angular_vel, 10)
#     isDriving = True
#     angle_tolerance = 0.1
#
#     if angle < 0:
#         angular_vel = -angular_vel
#
#     destination_angle = addAngles(current_theta, angle)
#
#     while isDriving:
#         print "Current theta: %f, Destination angle: %f, Angle: %f" % (current_theta, destination_angle, angle)
#
#         isAccelerating = current_theta > destination_angle + angle_tolerance \
#                          or current_theta < destination_angle - angle_tolerance
#
#         publishTwist(0, angularVelList[currentIndex])
#
#         if isAccelerating:
#             if currentIndex < len(angularVelList) - 2:
#                 if debug_mode:
#                     print "Accelerating: %f" % (angularVelList[currentIndex])
#                 currentIndex += 1
#             else:
#                 if debug_mode:
#                     print "Constant speed: %f" % (angularVelList[currentIndex])
#         else:
#             if currentIndex >= 0:
#                 if debug_mode:
#                     print "Decelerating: %f" % (angularVelList[currentIndex])
#                 currentIndex -= 1
#             else:
#                 isDriving = False
#
#         rospy.sleep(rospy.Duration(0, 1))
#
#     publishTwist(0, 0)

#Accepts an angle (rad) and makes the robot rotate around it.
def rotate(angle):
    pollRate = .1
    tolerance = .1

    # rospy.sleep(.5)

    # angle_in_rad = angle * (math.pi/180)
    thetaGoal = current_theta + angle

    #  so 0-2pi wraps around like 0-360 deg
    if thetaGoal >= math.pi:
	    thetaGoal = thetaGoal - 2*math.pi
    if thetaGoal <= -(math.pi):
        thetaGoal = thetaGoal + 2*math.pi

    print "current_theta: %f, goal theta: %f" %(current_theta, thetaGoal)

    while(current_theta < thetaGoal - tolerance or current_theta > thetaGoal + tolerance):

        if angle <= 0:
            publishTwist(0,-.5) # turn clockwise
        if angle > 0:
            publishTwist(0,.5) # turn counter-clockwise

        # print "current_theta: %f, goal theta: %f" %(current_theta, thetaGoal)

        # rospy.sleep(pollRate)

    publishTwist(0,0) #  stop robot

#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    angular_vel = speed / radius
    currentIndex = 0
    speedList = createSpeedList(speed, 10)
    angularVelList = createSpeedList(angular_vel, 10)
    isDriving = True
    angle_tolerance = 0.1


    if angle < 0:
        angular_vel = -angular_vel

    destination_angle = addAngles(current_theta, angle)

    while isDriving:
        isAccelerating = current_theta > destination_angle + angle_tolerance \
                         or current_theta < destination_angle - angle_tolerance

        publishTwist(speedList[currentIndex], angularVelList[currentIndex])

        if isAccelerating:
            if currentIndex < len(angularVelList) - 2:
                if debug_mode:
                    print "Accelerating: %f" % (angularVelList[currentIndex])
                currentIndex += 1
            else:
                if debug_mode:
                    print "Constant speed: %f" % (angularVelList[currentIndex])
        else:
            if currentIndex >= 0:
                if debug_mode:
                    print "Decelerating: %f" % (angularVelList[currentIndex])
                currentIndex -= 1
            else:
                isDriving = False

        rospy.sleep(rospy.Duration(0, 1))

    publishTwist(0, 0)

#Bumper Event Callback function
def readBumper(msg):
    if debug_mode:
        print "Bumper was pressed! State: %d, Bumper: %d" % (msg.state, msg.bumper)

    if msg.state == BumperEvent.PRESSED and msg.bumper == BumperEvent.CENTER:
        global bumperPressed
        bumperPressed = 1

def goToPosition(goalX, goalY):
    global current_x
    global current_y
    global current_theta

    xDiff = goalX - current_x
    yDiff = goalY - current_y

    # adding current_theta is done in rotate(angle)
    angle = math.atan2(xDiff, yDiff) - current_theta # x/y, not y/x

    rotate(angle)

    distance = math.sqrt(pow(xDiff, 2) + pow(yDiff, 2))
    print "distance: %f" %distance
    driveStraight(.1, distance)

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

    # print "read_odometry: X: %f, Y: %f, Theta: %f" % (current_x, current_y, current_theta)

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
    print "Entered sendRequest..."

    rospy.wait_for_service('calculateTrajectory')

    print "Finished wait_for_service..."

    try:
        calculateTraj = rospy.ServiceProxy('calculateTrajectory', Trajectory)
        trajectory = calculateTraj(initPos, goalPos)

        # for point in trajectory.path.poses:
        #     print "sendRequest: X: %f, Y: %f" % (point.pose.position.x, point.pose.position.y)

        return trajectory
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#This function sequentially calls methods to perform a trajectory.
def executeTrajectory(trajectory):
    waypoint_counter = 0

    for point in trajectory.path.poses:
        goalX = point.pose.position.x
        goalY = point.pose.position.y

        waypoint_counter = waypoint_counter + 1
        print "Going to Waypoint %d" %waypoint_counter
        goToPosition(goalX, goalY)

if __name__ == "__main__":
    rospy.init_node('control')

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




