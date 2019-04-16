# This class manages the key logic for the reactive planner and
# controller. This monitors the the robot motion.

import rospy
import threading
import time
from cell import CellLabel
from planner_controller_base import PlannerControllerBase
from comp0037_mapper.msg import *
from comp0037_reactive_planner_controller.aisle import Aisle

class ReactivePlannerController(PlannerControllerBase):

    def __init__(self, occupancyGrid, planner, controller):
        PlannerControllerBase.__init__(self, occupancyGrid, planner, controller)
        
        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)
        self.gridUpdateLock =  threading.Condition()
        self.aisleToDriveDown = None

        # Set the coordinates of aisles
        self.aisleCoords = {
            Aisle.A : (26, 32),
            Aisle.B : (43, 32),
            Aisle.C : (59, 32),
            Aisle.D : (75, 32),
            Aisle.E : (91, 32)
        }

        # Store the starting position
        self.start = (0, 0)

        # Store the current cost of robot
        self.currentCost = 0

        # wait time
        self.waitTime = 0

        # has waited?
        self.waited = False

    def mapUpdateCallback(self, mapUpdateMessage):

        # Update the occupancy grid and search grid given the latest map update
        self.gridUpdateLock.acquire()
        self.occupancyGrid.updateGridFromVector(mapUpdateMessage.occupancyGrid)
        self.planner.handleChangeToOccupancyGrid()
        self.gridUpdateLock.release()

        # If we are not currently following any route, drop out here.
        if self.currentPlannedPath is None:
            return

        self.checkIfPathCurrentPathIsStillGood()

    def checkIfPathCurrentPathIsStillGood(self):

        # This methods needs to check if the current path, whose
        # waypoints are in self.currentPlannedPath, can still be
        # traversed
                
        # If the route is not viable any more, call
        # self.controller.stopDrivingToCurrentGoal()

        waypoints = self.currentPlannedPath.waypoints
        for waypoint in waypoints:
            dX = waypoint.coords[0]
            dY = waypoint.coords[1]

            if self.occupancyGrid.getCell(dX, dY) == 1:
                self.controller.stopDrivingToCurrentGoal()
                break


    # Choose the first aisle the robot will initially drive down.
    # This is based on the prior.
    def chooseInitialAisle(self, startCellCoords, goalCellCoords):
        # Turn the graphics off for a sec
        self.planner.showGraphics = False

        lowestCost = 999999
        candidateAisle = None

        pathViaB = None
        pathViaC = None

        for aisle in Aisle:
            candidatePath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, aisle)
            print("============================================")
            print("path cost of " + str(aisle) + " is " + str(candidatePath.travelCost))        
            print("============================================")
            if candidatePath.travelCost < lowestCost:
                lowestCost = candidatePath.travelCost
                candidateAisle = aisle

            if aisle == Aisle.B:
                pathViaB = candidatePath
            
            if aisle == Aisle.C:
                pathViaC = candidatePath

        costDiff = pathViaC.travelCost - pathViaB.travelCost
        lamda = costDiff / 1.6 # 2 * 0.8 = 1.6

        print("============================================")
        print("minimum lamda is " + str(lamda) + " seconds")        
        print("============================================")


        self.planner.showGraphics = True
        return candidateAisle

    # Choose the subdquent aisle the robot will drive down
    def chooseAisle(self, startCellCoords, goalCellCoords):
        self.planner.showGraphics = False
        
        lowestCost = 999999
        candidateAisle = None

        for aisle in Aisle:
            if self.aisleToDriveDown == aisle:
                continue

            candidatePath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, aisle)
            if candidatePath.travelCost < lowestCost:
                lowestCost = candidatePath.travelCost
                candidateAisle = aisle

        self.planner.showGraphics = True
        return candidateAisle

    # Return whether the robot should wait for the obstacle to clear or not.
    def shouldWaitUntilTheObstacleClears(self, startCellCoords, goalCellCoords):
        # set the unit cost of waiting
        L_w = 2

        # Although not really possible, set the default aisle to B
        if self.aisleToDriveDown is None:
            self.aisleToDriveDown = Aisle.B
        
        # get the current path cost from start position to current position
        self.planner.search(startCellCoords, self.start)
        
        # Turn the graphics off for a sec
        self.planner.showGraphics = False
        
        workedThroughPath = self.planner.extractPathToGoal()        
        self.currentCost = workedThroughPath.travelCost

        remainCost = self.currentPlannedPath.travelCost - self.currentCost

        # Choose a new aisle
        newAisle = self.chooseAisle(startCellCoords, goalCellCoords)
        self.aisleToDriveDown = newAisle

        # Find the cost of new path
        newPath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, newAisle)
        newCost = newPath.travelCost

        # maximum wait time
        time = (newCost - remainCost) / L_w
        print("============================================")
        print("minimum lamda-b is " + str(time) + " seconds")        
        print("============================================")
        # Actual wait time: half of max
        self.waitTime = time / 2
        
        # Turn the graphic on again
        self.planner.showGraphics = True
        
        return True

    # This method will wait until the obstacle has cleared and the robot can move.
    def waitUntilTheObstacleClears(self):        
        time.sleep(self.waitTime)
        print("============================================")
        print("Has waited for " + str(self.waitTime) + " seconds")
        print("============================================")
        self.waitTime = 0
    
    # Plan a path to the goal which will go down the designated aisle. The code, as
    # currently implemented simply tries to drive from the start to the goal without
    # considering the aisle.
    def planPathToGoalViaAisle(self, startCellCoords, goalCellCoords, aisle):

        # Note that, if the robot has waited, it might be tasked to drive down the
        # aisle it's currently on. Your code should handle this case.
        if self.aisleToDriveDown is None:
            self.aisleToDriveDown = aisle

        # Implement your method here to construct a path which will drive the robot
        # from the start to the goal via the aisle.        
        '''
        pathToGoalFound = self.planner.search(startCellCoords, goalCellCoords) 

        # If we can't reach the goal, give up and return
        if pathToGoalFound is False:
            rospy.logwarn("Could not find a path to the goal at (%d, %d)", \
                            goalCellCoords[0], goalCellCoords[1])
            return None

        # Extract the path
        currentPlannedPath = self.planner.extractPathToGoal()

        return currentPlannedPath
        '''
        # Get the coordinate of aisle
        aisleCoord = self.aisleCoords[aisle]
        
        # Get the path to the aisle
        startToAisle = self.planner.search(startCellCoords, aisleCoord) 
        if startToAisle is False:
            rospy.logwarn("Could not find a path to the aisle at (%d, %d)", \
                            aisleCoord[0], aisleCoord[1])
            return None
        
        pathToCurrentAisle = self.planner.extractPathToGoal()
                
        # Get the path from the aisle to the goal
        aisleToGoal = self.planner.search(aisleCoord, goalCellCoords)
        if aisle is False:
            rospy.logwarn("Could not find a path from aisle to the goal at (%d, %d)", \
                            goalCellCoords[0], goalCellCoords[1])
            return None
        
        pathToCurrentGoal = self.planner.extractPathToGoal()

        # Concatinate the paths and get the full path
        pathToCurrentAisle.addToEnd(pathToCurrentGoal)
        currentPlannedPath = pathToCurrentAisle
        
        # redraw the path
        if self.planner.showGraphics == True:
            self.planner.searchGridDrawer.update()
            self.planner.searchGridDrawer.drawPathGraphicsWithCustomColour(currentPlannedPath, 'yellow')
            self.planner.searchGridDrawer.waitForKeyPress()


        return currentPlannedPath

    # This method drives the robot from the start to the final goal. It includes
    # choosing an aisle to drive down and both waiting and replanning behaviour.
    # Note that driving down an aisle is like introducing an intermediate waypoint.

    def driveToGoal(self, goal):

        # Get the goal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        # Set the start conditions to the current position of the robot
        pose = self.controller.getCurrentPose()
        start = (pose.x, pose.y)
        startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

        self.start = startCellCoords

        # Work out the initial aisle to drive down
        self.aisleToDriveDown = self.chooseInitialAisle(startCellCoords, goalCellCoords)

        # Reactive planner main loop - keep iterating until the goal is reached or the robot gets
        # stuck.
        
        while rospy.is_shutdown() is False:

            # Plan a path from the robot's current position to the goal. This is called even
            # if the robot waited and used its existing path. This is more robust than, say,
            # stripping cells from the existing path.           
            
            print 'Planning a new path: start=' + str(start) + '; goal=' + str(goal)
            
            # Plan a path using the current occupancy grid
            self.gridUpdateLock.acquire()
            self.currentPlannedPath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, self.aisleToDriveDown)
            self.gridUpdateLock.release()

            # If we couldn't find a path, give up
            if self.currentPlannedPath is None:
                return False

            # Drive along the path towards the goal. This returns True
            # if the goal was successfully reached. The controller
            # should stop the robot and return False if the
            # stopDrivingToCurrentGoal method is called.
            goalReached = self.controller.drivePathToGoal(self.currentPlannedPath, \
                                                          goal.theta, self.planner.getPlannerDrawer())

            rospy.logerr('goalReached=%d', goalReached)

            # If we reached the goal, return
            if goalReached is True:
                return True

            # An obstacle blocked the robot's movement. Determine whether we need to
            # wait or replan.

            # Figure out where we are
            pose = self.controller.getCurrentPose()
            start = (pose.x, pose.y)
            startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

            # See if we should wait
            waitingGame = self.shouldWaitUntilTheObstacleClears(startCellCoords, goalCellCoords)

            # Depending upon the decision, either wait or determine the new aisle
            # we should drive down.
            if waitingGame is True:
                self.waitUntilTheObstacleClears()
            else:
                aisleToDriveDown = self.chooseAisle(startCellCoords, goalCellCoords)

        return False
            
            
