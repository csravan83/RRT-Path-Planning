import pygame
from pygame.locals import *
import time
import random
import numpy as np
from random import shuffle
import sys
import cv2
from copy import copy


# GLOBAL FUNCTIONS

# cumulative dot product for a list of numpy arrays
def CumulDot(MList, i):
    if i == 0:
        Cdot = MList[0]
    else:
        Cdot = MList[0]
        for j in range(1, i + 1):
            Cdot = np.dot(Cdot, MList[j])
    return Cdot


# Rounds value to the nearest N integer
def round_N(val, N):
    dev = 0.5 if N >= 0 else -0.5
    return int(val / N + dev) * N


# Sets allowable rotation range for angle configurations
# Converts angles > max to a value between max and min
def setRange(angle, max, min):
    if angle > max or angle < min:
        angle360 = angle % max
    elif angle < min:
        angle360 = angle % min
    else:
        angle360 = angle

    return angle360


# Call to the Pygame event handler.  Used to escape potential infinite loops during debugging
def handleEvents():
    for event in pygame.event.get():
        if event.type == KEYUP and event.key == K_ESCAPE:
            pygame.quit()
            sys.exit()
            #		elif event.type == pygame.MOUSEMOTION:
            #			goal = np.array(list(arm.PointTrue((event.pos)))[:3])
    return


class ArmPlanning():
    def __init__(self, length, angles, goal, colors):
        pygame.init()
        self.screen = pygame.display.set_mode((DISP_WIDTH, DISP_HEIGHT))
        self.screen.fill((255, 255, 255))

        self.length = length
        self.angles = angles  # joint angles
        self.colors = colors  # the color for each link
        self.dof = len(angles)  # degree of freedom

        self.points = [self.PointDisplay([0, 0])] * (self.dof + 1)  # shift to the center
        self.eloc = self.PointTrue(self.points[-1])[:3]  # effector location in x,y,z coordinates

        self.obstacles = []
        self.lines = [pygame.rect.Rect(0, 0, 0, 0)] * self.dof
        self.obstacle_rects = []

        # defines the transformation matrices for the arm
        c = np.cos(angles)
        s = np.sin(angles)
        self.A = [np.array(
            [[c[i], -s[i], 0, length[i] * c[i]], [s[i], c[i], 0, length[i] * s[i]], [0, 0, 1, 0], [0, 0, 0, 1]]) for i
                  in range(len(length))]
        self.T = [CumulDot(self.A, i) for i in range(len(self.A))]

        # RRT Planning Stuff
        self.start = None
        self.end = None
        self.graph = []
        self.randList = []
        self.startTree = {}
        self.endTree = {}
        self.intersection = None  # coordinates where trees intersect
        self.path = []  # list of configurations which map the found path

        # used to store screen captures
        self.images = []
        self.imagecheck = True

    # Draws the arm in its current configuration in Pygame
    def draw(self):
        self.screen.fill((255, 255, 255))  # clean the screen first
        # draw the whole arm
        self.lines = [pygame.draw.line(self.screen, colors[i], self.points[i], self.points[i + 1], 5) for i in
                      range(0, self.dof)]

        # draw the obstacles
        for i in range(0, len(self.obstacles)):
            self.obstacle_rects[i] = pygame.draw.circle(self.screen, self.obstacles[i][0], self.obstacles[i][1],
                                                        self.obstacles[i][2], 0)

        pygame.display.update()

        if self.imagecheck == True:
            # Take a screen capture of the current configuration and saves it as a 2D numpy array
            # NOTE:This slows system performance considerably.  Comment out for faster operation.
            screenshot = pygame.display.get_surface()
            np_image = pygame.surfarray.array3d(screenshot)
            self.images.append(np_image)

    def addObstacle(self, color, loc, radius):
        obs = (color, self.PointDisplay([loc[0], loc[1]]), radius)
        self.obstacles.append(obs)
        self.obstacle_rects.append(pygame.rect.Rect(0, 0, 0, 0))

    # Convert a point, (x,y) Tuple, to PyGame coordinate
    def PointDisplay(self, point):
        return (DISP_WIDTH / 2 + point[0], DISP_HEIGHT - point[1])

    # Converts an (x,y) tuple to world coordinates (flip of pygame coordinates)
    def PointTrue(self, (x, y)):
        return [x - DISP_WIDTH / 2, DISP_HEIGHT - y, 0.0, 1.0]

    # Forward kinematics method that updates (x,y) joint locations from specified joint configurations
    def fKinematics(self, angles):
        self.angles = angles

        c = np.cos(angles)
        s = np.sin(angles)
        self.A = [np.array(
            [[c[i], -s[i], 0, self.length[i] * c[i]], [s[i], c[i], 0, self.length[i] * s[i]], [0, 0, 1, 0],
             [0, 0, 0, 1]]) for i in range(len(self.length))]
        self.T = [CumulDot(self.A, i) for i in range(len(self.A))]

        old_pts = [self.PointTrue(point) for point in self.points]
        new_pts = [np.dot(self.T[i], old_pts[0]).tolist() for i in range(len(self.T))]
        new_pts.insert(0, old_pts[0])

        return new_pts  # new locations in WORLD space

    # Inverse kinematics via the inverse Jacobian method. Uses Moore-Penrose pseudoinverse method
    def iKinematics(self, goal):

        J = self.getJacobian()

        JI = np.linalg.pinv(J)
        goal3D = np.hstack((goal, [0, 0, 1]))
        loc3D = np.hstack((self.eloc, [0, 0, 1]))
        de = BETA * (goal3D - loc3D)
        dt = np.dot(JI, de)
        theta = arm.angles + dt

        return theta

    # Calculates the Jacobian of a 2D system
    def getJacobian(self):
        ai = np.array([0, 0, 1])  # unit length rotation axis in WORLD space (always z-axis for 2D problem)
        e = np.asarray(self.eloc)

        dedt = [np.cross(ai, e - np.asarray(self.PointTrue(point[:3])[:3])) for point in self.points[0:self.dof]]

        J1 = np.array(dedt).transpose()
        J2 = np.array([ai] * self.dof).transpose()
        J = np.vstack((J1, J2))
        return J

    # RRT Planning Algorithm which uses an implicit map discretization
    def RRTExtend2(self, someTree, goalNode):
        distances = dict((node, np.linalg.norm(np.asarray(node) - np.asarray(goalNode))) for node in someTree)
        nearestNode = min(distances, key=distances.get)
        delta = (np.asarray(goalNode) - np.asarray(nearestNode)) / np.linalg.norm(
            np.asarray(nearestNode) - np.asarray(goalNode))
        newNode = tuple(nearestNode)
        distance = np.linalg.norm(np.asarray(newNode) - np.asarray(goalNode))
        while distance > RRTTOLERANCE:
            handleEvents()

            lastNode = newNode
            newNode = newNode + delta
            distance = np.linalg.norm(np.asarray(newNode) - np.asarray(goalNode))

            # check if newest configuration is in collision with an obstacle
            check = self.collisionCheck(newNode)
            if check == True:
                return

            # this handles the case where roundoff error makes lastNode and newNode equal
            if tuple(np.round(lastNode)) != tuple(np.round(newNode)):
                someTree[tuple(np.round(newNode))] = tuple(np.round(lastNode))

            # if trees intersect, end the tree extension
            self.FindIntersections()
            if self.intersection != None:
                print 'intersection found at: ' + str(self.intersection)
                return
        return

    def FindIntersections(self):
        for key in self.startTree:
            if key in self.endTree:
                self.intersection = key

    def BuildPath(self):
        print 'building paths to/from:' + str(self.intersection)
        self.path = []
        startPath = []
        endPath = []

        # find the path from intersection back to start
        node = self.intersection
        print 'building start path'
        while node != self.start:
            startPath.append(node)
            node = self.startTree[node]  # moves one node back in path

            handleEvents()

        startPath.reverse()  # reverse path so it begins at start
        startPath.pop()  # removes meetingNode from startPath (will be included in endPath)

        # find the path from intersection to end
        node = self.intersection
        print 'building end path'
        while node != self.end:
            endPath.append(node)
            node = self.endTree[node]  # moves one node back in path

            handleEvents()

        self.path = startPath + endPath

    # check each link to see if it collides with an obstacle
    def collisionCheck(self, config):
        # get the joint locations of the configuration (in PYGAME space)
        config_pts = self.fKinematics(np.radians(config))
        config_pts = [self.PointDisplay(point) for point in config_pts]

        links = [pygame.draw.line(self.screen, colors[i], config_pts[i], config_pts[i + 1], 5) for i in
                 range(0, self.dof)]

        for link in links:
            if (link.collidelist(self.obstacle_rects[1:]) != -1):
                return True
        return False


# -------------------------------------Main Program----------------------------------------------
# the units are (cm or pixel), and (degree) 
# The coordinate of PyGame is:
#    ---> x
#    '
#    '
#    y
# the upperleft pixel is (0,0)
# Note: use function PointDisplay() to convert to PyGame coordinate
#       use function PointActual()  to convert to the actual coordinate

# The size of the display window
DISP_WIDTH = 500
DISP_HEIGHT = 300

length = [100, 50, 50, 0]
init_angles = [-25, -25, -25, 0]
colors = [(0, 0, 255), (255, 0, 0), (0, 255, 0), (0, 255, 0)]
goal = np.array([0, 200, 0])

# allowable angle range for arm
tmax = 180
tmin = -180

# Other calibration constants
BETA = 0.05  # step size for IK solution
GRID = 5  # configuration space discretization
IKTOLERANCE = 5  # tolerance threshold for IK solution convergence
RRTTOLERANCE = 1  # tolerance threshold for RRT "extend" function to be at goal node

print '------------------------APPLICATION DEBUG COMMENTS--------------------'
print 'Arm is allowed to explore configurations that are "off screen"'
print '------------------------------END COMMENTS----------------------------'
print '---------------------------BEGIN MAIN PROGRAM-------------------------'

if __name__ == "__main__":

    arm = ArmPlanning(length, np.radians(init_angles), goal, colors)

    # this sets the goal
    arm.addObstacle((0, 255, 0), goal.tolist(), int(12.5))

    # these are additional obstacles
    arm.addObstacle((255, 105, 180), [-100, 100, 0], int(7))
    arm.addObstacle((255, 105, 180), [100, 100, 0], int(7))
    # arm.addObstacle( (255,105,180), [-50,100,0] , int(5))
    # arm.addObstacle( (255,105,180), [50,100,0] , int(5))

    arm.draw()

    if arm.imagecheck:
        print 'WARNING: storing each screen shot as image! This will result in significant slowdown. Set imagecheck flag to False to disable screen captures '

    angleRange = np.arange(tmin, tmax, GRID)
    configMap = [(t1, t2, t3, 0) for t1 in np.arange(tmin, tmax, GRID) for t2 in np.arange(tmin, tmax, GRID) for t3 in
                 np.arange(tmin, tmax, GRID)]

    while True:
        i = 0
        handleEvents()

        # first, find the start and goal configurations
        distance = np.linalg.norm(goal - np.asarray(arm.eloc))
        print 'Finding goal configuration via Jacobian inverse kinematics'

        while distance > IKTOLERANCE:

            handleEvents()

            # given goal in WORLD space, find the corresponding configuration in joint space
            new_config = arm.iKinematics(goal)

            # given new configuration in joint space, move joints in WORLD space
            new_pts = arm.fKinematics(new_config)

            # update joint locations (stored in display coordinates)
            arm.points = [arm.PointDisplay(point) for point in new_pts]

            arm.draw()

            # update end effector location and distance from goal (stored in WORLD coordinates)
            arm.eloc = arm.PointTrue(arm.points[-1])[:3]
            distance = np.linalg.norm(goal - np.asarray(arm.eloc))

            # keep track of iteration counts to prevent infinite loop conditions
            i += 1
            if i > 500:
                print 'failed to converge'
                break

        # take found configurations and plan using RRT
        print 'Path planning via RRT'

        # Initializations
        arm.start = tuple([float(round_N(angle, GRID)) for angle in init_angles])

        arm.end = [setRange(angle, tmax, tmin) for angle in np.degrees(arm.angles)]
        arm.end = [round_N(angle, GRID) for angle in arm.end]
        arm.end[3] = 0
        arm.end = tuple(arm.end)

        print 'Start Configuration: ' + str(arm.start)
        print 'Goal Configuration: ' + str(arm.end)

        arm.startTree[arm.start] = ''
        arm.endTree[arm.end] = ''
        arm.graph = configMap  # graph is a list of coordinate tuples
        arm.randList = copy(configMap)

        # Build list of node IDs in map, shuffle it randomly
        shuffle(arm.randList)
        arm.randList.remove(arm.start)
        arm.randList.remove(arm.end)

        while arm.randList:

            handleEvents()

            randomNode = arm.randList.pop()

            # print 'RRT Goal Node: ' + str(randomNode)

            arm.RRTExtend2(arm.startTree, randomNode)
            arm.RRTExtend2(arm.endTree, randomNode)

            if arm.intersection != None:
                #
                arm.BuildPath()
                print 'start of path: ' + str(arm.start)
                print 'end of path: ' + str(arm.end)
                break

        print 'plannning complete'

        # animate the arm moving from start to goal configurations using path found from RRT
        while arm.path:
            newConfig = list(arm.path.pop(0))
            new_pts = arm.fKinematics(np.radians(newConfig))
            arm.points = [arm.PointDisplay(point) for point in new_pts]
            arm.draw()

        # make a video from image sequence using opencv
        img1 = arm.images[0]
        fps = 20
        width = img1.shape[0]
        height = img1.shape[1]
        fourcc = cv.CV_FOURCC('D', 'I', 'V', 'X')  # mpeg4 video codec
        writer = cv2.VideoWriter('out.avi', fourcc, fps, (width, height))

        for image in arm.images:
            imgfinal = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
            imgfinal[:, :, 0] = image[:, :, 2] * 1
            imgfinal[:, :, 1] = image[:, :, 1] * 1
            imgfinal[:, :, 2] = image[:, :, 0] * 1
            imgfinal = np.swapaxes(imgfinal, 0, 1)
            writer.write(imgfinal)
        print 'video finished'
        print 'Program complete. Press ESCAPE to exit'

        while True:
            handleEvents()

