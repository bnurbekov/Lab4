#!/usr/bin/env python

import rospy, time, math, Queue
from kobuki_msgs.msg import BumperEvent
# Add additional imports for each of the message types used
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, GridCells, Path
from lab4.srv import *

#A class that has a function of cell type enumeration.
class CellType:
    Unexplored, Expanded, Frontier, Obstacle = range(4)

#A class that contains logic and properties related to the cell on the board.
class Cell:
    def __init__(self, coordinate, cellType=CellType.Unexplored):
        self.coordinate = coordinate
        self.type = cellType

    def __cmp__(self, other):
        if self.totalCost > other.totalCost:
            return 1
        elif self.totalCost < other.totalCost:
            return -1
        else:
            return 0

#A class that contains logic and properties related to the cellCoordinate on the board.
class CellCoordinate:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    #Checks if the cell coordinate is within the board.
    def isWithinBoard(self, boardWidth, boardHeight):
        return (self.x <= boardWidth-1 and self.x >= 0) and \
               (self.y <= boardHeight-1 and self.y >= 0)

    #Estimates the heurestic.
    def getHeuristic(self, destinationCellCoordinate):
        return math.sqrt(math.pow(self.x - destinationCellCoordinate.x, 2) +
                         math.pow(self.y - destinationCellCoordinate.y, 2))

    #Returns the path cost for the adjacent cells. Warning: the function will throw an exception, if cells are not adjacent
    def getPathCost(self, cellCoordinate):
        xDiff = abs(self.x - cellCoordinate.x)
        yDiff = abs(self.y - cellCoordinate.y)

        if (xDiff > 1 or xDiff < 0) or (yDiff > 1 or yDiff < 0):
            raise Exception("getPathCost: The function estimates the cost only for adjacent cells!")

        if xDiff + yDiff == 2:
            return 1.4
        elif xDiff + yDiff == 1:
            return 1
        else:
            return 0

    #Returns the indication of whether the cell is valid neighbor or not
    def isValidNeighbor(self, neigborCellCoordinate):
        xDiff = abs(self.x - neigborCellCoordinate.x)
        yDiff = abs(self.y - neigborCellCoordinate.y)

        return neigborCellCoordinate.isWithinBoard(GRID_WIDTH, GRID_HEIGHT) \
               and not self.__eq__(neigborCellCoordinate) \
               and (xDiff <= 1 and xDiff >= 0) and (yDiff <= 1 and yDiff >= 0)

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other):
        return (self.x == other.x) and (self.y == other.y)

#Callback function that processes the OccupancyGrid message.
def mapCallback(gridMessage):
    global originalGridMessage
    global wasMapReceived

    if not wasMapReceived:
        originalGridMessage = gridMessage

        wasMapReceived = True

#Processes the received occupancy grid message.
def processOccupancyGrid():
    #extract the position of the center of the occupancy grid
    global originalGridMessage
    global grid
    global GRID_WIDTH
    global GRID_HEIGHT
    global cellOriginX
    global cellOriginY

    grid = {}

    GRID_HEIGHT = originalGridMessage.info.height
    GRID_WIDTH = originalGridMessage.info.width

    cellOriginX = originalGridMessage.info.origin.position.x + originalGridMessage.info.resolution/2
    cellOriginY = originalGridMessage.info.origin.position.y + originalGridMessage.info.resolution/2

    populateGrid()

    expandObstacles()

def populateGrid():
    global grid

    counter = 0

    for i in range(0, originalGridMessage.info.height):
        for j in range(0, originalGridMessage.info.width):
            if originalGridMessage.data[counter] == 100:
                tempCell = Cell(CellCoordinate(j, i), cellType=CellType.Obstacle)
            else:
                tempCell = Cell(CellCoordinate(j, i), cellType=CellType.Unexplored)

            grid[tempCell.coordinate] = tempCell

            counter += 1

#Expands the obstacles
def expandObstacles():
    obstacleCells = []

    for cellKey in grid:
        cell = grid[cellKey]
        if cell.type == CellType.Obstacle:
            obstacleCells.append(cell)

    for obstacleCell in obstacleCells:
        for i in range(0, 3):
            for j in range(0, 3):
                neigborCellCoordinate = CellCoordinate((obstacleCell.coordinate.x - 1) + j, (obstacleCell.coordinate.y - 1) + i)

                if (obstacleCell.coordinate.isValidNeighbor(neigborCellCoordinate)):
                    grid[neigborCellCoordinate].type = CellType.Obstacle

#Callback function that processes the initial position received.
def processInitPos(initPos):
    global wasInitPosReceived
    global initPosCellCoordinate

    initPosCellX = (initPos.pose.pose.position.x - originalGridMessage.info.origin.position.x) // originalGridMessage.info.resolution
    initPosCellY = (initPos.pose.pose.position.y - originalGridMessage.info.origin.position.y) // originalGridMessage.info.resolution

    print "Initial cell: X(%f), Y(%f)" % (initPosCellX, initPosCellY)

    tempCellCoordinate = CellCoordinate(initPosCellX, initPosCellY)

    if not wasInitPosReceived:
        if not tempCellCoordinate.isWithinBoard(GRID_WIDTH, GRID_HEIGHT):
            raise Exception("Error: The position selected was outside of the grid! Please, try again.")
        else:
            #we can store the init pos as the global variable
            initPosCellCoordinate = tempCellCoordinate
            wasInitPosReceived = True

#Adds neighbors as frontiers on the grid.
def addNeighborsAsFrontiers(parentCell):
    #Add neighbors to the priority queue and set parent
    for i in range(0, 3):
        for j in range(0, 3):
            currentCellCoordinate = CellCoordinate((parentCell.coordinate.x - 1) + j, (parentCell.coordinate.y - 1) + i)

            if (parentCell.coordinate.isValidNeighbor(currentCellCoordinate)):
                    currentCell = grid[currentCellCoordinate]

                    newPathCost = parentCell.pathCost + parentCell.coordinate\
                            .getPathCost(currentCellCoordinate)

                    if (currentCell.type == CellType.Frontier):
                        if currentCell.pathCost > newPathCost:
                            currentCell.parent = parentCell
                            currentCell.pathCost = newPathCost
                            currentCell.totalCost = newPathCost + currentCellCoordinate.getHeuristic(goalPosCellCoordinate)
                            frontierList.sort()
                    elif (currentCell.type == CellType.Unexplored):
                        currentCell.parent = parentCell
                        currentCell.type = CellType.Frontier
                        currentCell.pathCost = newPathCost
                        currentCell.totalCost = newPathCost + currentCellCoordinate.getHeuristic(goalPosCellCoordinate)
                        frontierList.append(currentCell)
                        frontierList.sort()

#Callback function that processes the initial position received.
def processGoalPos(goalPos):
    global wasGoalPosReceived
    global goalPosCellCoordinate

    goalPosCellX = (goalPos.pose.position.x - originalGridMessage.info.origin.position.x) // originalGridMessage.info.resolution
    goalPosCellY = (goalPos.pose.position.y - originalGridMessage.info.origin.position.y) // originalGridMessage.info.resolution

    print "Goal cell: X(%f), Y(%f)" % (goalPosCellX, goalPosCellY)

    tempCellCoordinate = CellCoordinate(goalPosCellX, goalPosCellY)

    if not wasGoalPosReceived:
        if not tempCellCoordinate.isWithinBoard(GRID_WIDTH, GRID_HEIGHT):
            raise Exception("Error: The position selected was outside of the grid! Please, try again.")
        else:
            goalPosCellCoordinate = CellCoordinate(goalPosCellX, goalPosCellY)
            wasGoalPosReceived = True

#Creates a GridCells message and auto-initializes some fields.
def createGridCellsMessage():
    gridCells = GridCells()
    gridCells.header.seq = 0
    gridCells.header.stamp = rospy.Time.now()
    gridCells.header.frame_id = "map"
    gridCells.cell_width = originalGridMessage.info.resolution
    gridCells.cell_height = originalGridMessage.info.resolution

    gridCells.cells = []

    return gridCells

#Publishes the grid as GridCells for RViz.
def publishGridCells():
    unexploredGridCells = createGridCellsMessage()
    frontierGridCells = createGridCellsMessage()
    expandedGridCells = createGridCellsMessage()

    for i in range(0, originalGridMessage.info.height):
        for j in range(0, originalGridMessage.info.width):
            point = Point()
            point.x = cellOriginX + originalGridMessage.info.resolution*j
            point.y = cellOriginY + originalGridMessage.info.resolution*i

            tempCellCoordinate = CellCoordinate(j, i)

            if grid[tempCellCoordinate].type == CellType.Unexplored:
                unexploredGridCells.cells.append(point)
            elif grid[tempCellCoordinate].type == CellType.Frontier:
                frontierGridCells.cells.append(point)
            elif grid[tempCellCoordinate].type == CellType.Expanded:
                expandedGridCells.cells.append(point)

    unexplored_cell_pub.publish(unexploredGridCells)
    frontier_cell_pub.publish(frontierGridCells)
    expanded_cell_pub.publish(expandedGridCells)

#Prints the grid (primarily used for debugging).
def printGrid():
    for i in range(0, originalGridMessage.info.height):
        for j in range(0, originalGridMessage.info.width):
            cellCoordinate = CellCoordinate(j, 36 - i)
            print grid[cellCoordinate].type,
        print " "

#initializes the A* algorithm
def initAStar():
    initCell = grid[initPosCellCoordinate]
    initCell.type = CellType.Expanded

    if initCell.coordinate == goalPosCellCoordinate:
        return True

    initCell.pathCost = 0

    addNeighborsAsFrontiers(initCell)

    return False

#Runs an iteration of A* algorithm
def runAStarIteration():
    if len(frontierList) == 0:
        raise Exception("Error: Was not able to find the path to the destination!")

    nextParentCell = frontierList.pop(0)
    nextParentCell.type = CellType.Expanded

    if nextParentCell.coordinate == goalPosCellCoordinate:
        return True

    addNeighborsAsFrontiers(nextParentCell)

    return False

def clearRvizGrid():
    unexploredGridCells = createGridCellsMessage()
    frontierGridCells = createGridCellsMessage()
    expandedGridCells = createGridCellsMessage()

    unexplored_cell_pub.publish(unexploredGridCells)
    frontier_cell_pub.publish(frontierGridCells)
    expanded_cell_pub.publish(expandedGridCells)

def findPath():
    global pathCellCoordinateList
    pathCellCoordinateList = []

    currentCellCoordinate = goalPosCellCoordinate

    while not currentCellCoordinate.__eq__(initPosCellCoordinate):
        pathCellCoordinateList.append(currentCellCoordinate)
        currentCellCoordinate = grid[currentCellCoordinate].parent.coordinate

    pathCellCoordinateList.append(currentCellCoordinate)

def publishPath():
    pathGridCells = createGridCellsMessage()

    for cellCoordinate in pathCellCoordinateList:
        point = Point()
        point.x = cellOriginX + originalGridMessage.info.resolution*cellCoordinate.x
        point.y = cellOriginY + originalGridMessage.info.resolution*cellCoordinate.y

        pathGridCells.cells.append(point)

    path_cell_pub.publish(pathGridCells)

def calculateWaypoints():
    global waypoints

    pathCellCoordinateList.reverse()

    if len(pathCellCoordinateList) < 1:
        raise Exception("Error: Cannot extract waypoints from the empty path!")

    waypoints = []

    lastXDiff = pathCellCoordinateList[1].x - pathCellCoordinateList[0].x
    lastYDiff = pathCellCoordinateList[1].y - pathCellCoordinateList[0].y

    for i in range(1, len(pathCellCoordinateList) - 1):
        xDiff = pathCellCoordinateList[i+1].x - pathCellCoordinateList[i].x
        yDiff = pathCellCoordinateList[i+1].y - pathCellCoordinateList[i].y

        if lastXDiff != xDiff or lastYDiff != yDiff:
            waypoints.append(pathCellCoordinateList[i])
            lastXDiff = xDiff
            lastYDiff = yDiff

    waypoints.append(pathCellCoordinateList[len(pathCellCoordinateList) - 1])

def handleRequest(req):
    global waypoints

    processOccupancyGrid()

    processInitPos(req.initPos)
    processGoalPos(req.goalPos)

    aStarDone = initAStar()

    while not aStarDone:
        #run iteration
        aStarDone = runAStarIteration()

    findPath()

    publishPath()

    calculateWaypoints()

    print "Waypoints:"
    for cellCoordinate in waypoints:
        print cellCoordinate.x, cellCoordinate.y

    #convert the waypoints to the trajectory offsets:
    path = Path()
    path.poses = []

    for cellCoordinate in waypoints:
        poseObj = PoseStamped()
        poseObj.pose.position.x = cellOriginX + cellCoordinate.x*originalGridMessage.info.resolution
        poseObj.pose.position.y = cellOriginY + cellCoordinate.y*originalGridMessage.info.resolution
        poseObj.pose.position.z = 0

        path.poses.append(poseObj)

    resetVariables()

    return TrajectoryResponse(path)

#resets all the variables
def resetVariables():
    global wasInitPosReceived
    global wasGoalPosReceived
    global frontierList
    global aStarDone
    global grid
    global waypoints

    wasInitPosReceived = False
    wasGoalPosReceived = False
    aStarDone = False

    del frontierList[:]
    frontierList = []

    del waypoints[:]
    waypoints = []

    grid.clear()

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('lab4_path_planner')

    global unexplored_cell_pub
    global expanded_cell_pub
    global frontier_cell_pub
    global wasMapReceived
    global wasInitPosReceived
    global wasGoalPosReceived
    global frontierList
    global aStarDone
    global debugMode

    frontierList = []
    wasMapReceived = False
    wasInitPosReceived = False
    wasGoalPosReceived = False
    aStarDone = False
    debugMode = True

    map_sub = rospy.Subscriber('/map', OccupancyGrid, mapCallback, queue_size=1)
    unexplored_cell_pub = rospy.Publisher('/unexploredGridCells', GridCells, queue_size=5) # Publisher for commanding robot motion
    expanded_cell_pub = rospy.Publisher('/expandedGridCells', GridCells, queue_size=5) # Publisher for commanding robot motion
    frontier_cell_pub = rospy.Publisher('/frontierGridCells', GridCells, queue_size=5) # Publisher for commanding robot motion
    path_cell_pub = rospy.Publisher('/pathGridCells', GridCells, queue_size=5)

    # Use this command to make the program wait for some seconds
    # rospy.sleep(rospy.Duration(1, 0))

    print "Starting Lab 4"

    print "Waiting for the occupancy grid..."

    #wait for the initial position to be received
    while not wasMapReceived:
        pass

    print "Received initial occupancy grid!"

    s = rospy.Service('calculateTrajectory', Trajectory, handleRequest)
    print "Service is active now!"
    rospy.spin()

    # wait for some time to make sure that subscribers get the message

    print "Lab 4 complete!"
