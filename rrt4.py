import math, sys, pygame, random
from math import *
from pygame import *


class Node(object):
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent


XDIM = 900
YDIM = 720
windowSize = [XDIM, YDIM]
EPSILON = 5.0
OBS_TYPE = 2
GOAL_RADIUS = 10
#MIN_DISTANCE_TO_ADD = 1.0
NUMNODES = 50000
pygame.init()
fpsClock = pygame.time.Clock()
screen = pygame.display.set_mode(windowSize)
darkgreen = 47,79,79
black = 0, 0, 0
red = 255, 0, 0
blue = 0, 255, 0
green = 0, 0, 255
cyan = 0, 180, 105

count = 0
rectObs = []


def dist(p1, p2):  # distance between two points
    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))


def point_circle_collision(p1, p2, radius):
    distance = dist(p1, p2)
    if (distance <= radius):
        return True
    return False

def step_from_to(p1, p2):
    if dist(p1, p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        return p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)


def collides(p):  # check if point collides with the obstacle
    for rect in rectObs:
        if rect.collidepoint(p) == True:
            return True
    return False


def get_random_clear():
    while True:
        p = random.random() * XDIM, random.random() * YDIM
        noCollision = collides(p)
        if noCollision == False:
            return p


def init_obstacles(configNum):  # initialized the obstacle
    global rectObs
    rectObs = []
    print("config " + str(configNum))
    if (configNum == 0):
        rectObs.append(pygame.Rect((XDIM / 2.0 - 50, YDIM / 2.0 - 100), (100, 200)))
    if (configNum == 1):
        rectObs.append(pygame.Rect((100, 50), (200, 150)))
        rectObs.append(pygame.Rect((400, 200), (200, 120)))
        #rectObs.append(pygame.Rect((10,50), (100,50)))
    if (configNum == 2):
        rectObs.append(pygame.Rect((100, 50), (200, 150)))
        rectObs.append(pygame.Rect((400,200), (150, 150)))
        rectObs.append(pygame.Rect((170,300), (150,120)))
        rectObs.append(pygame.Rect((575,70),(100,100)))
        rectObs.append(pygame.Rect((659,414),(200,150)))
        rectObs.append(pygame.Rect((470,470),(150,120)))
        rectObs.append(pygame.Rect((720,220),(150,100)))
        rectObs.append(pygame.Rect((120,500),(100,100)))
        rectObs.append(pygame.Rect((320,519),(90,90)))


    if (configNum == 3):
        rectObs.append(pygame.Rect((100, 50), (200, 150)))

    for rect in rectObs:
        pygame.draw.rect(screen, black, rect)

def reset():
    global count
    screen.fill(darkgreen)
    init_obstacles(OBS_TYPE)
    count = 0


def main():
    global count

    initPoseSet = False
    initialPoint = Node(None, None)
    goalPoseSet = False
    goalPoint = Node(None, None)
    currentState = 'init'

    nodes = []
    reset()

    while True:
        if currentState == 'init':
            #print('goal point not yet set')
            pygame.display.set_caption('Select Starting Point and then Goal Point')
            fpsClock.tick(10)
        elif currentState == 'goalFound':
            currNode = goalNode.parent
            pygame.display.set_caption('Goal Reached')
            #print "Goal Reached"

            while currNode.parent != None:
                pygame.draw.line(screen, red, currNode.point, currNode.parent.point)
                currNode = currNode.parent
            optimizePhase = True
        elif currentState == 'optimize':
            fpsClock.tick(0.5)
            pass
        elif currentState == 'buildTree':
            count = count + 1
            pygame.display.set_caption('Performing RRT')
            if count < NUMNODES:
                foundNext = False
                while foundNext == False:
                    rand = get_random_clear()
                    parentNode = nodes[0]
                    for p in nodes:
                        if dist(p.point, rand) <= dist(parentNode.point, rand):
                            newPoint = step_from_to(p.point, rand)
                            if collides(newPoint) == False:
                                parentNode = p
                                foundNext = True

                newnode = step_from_to(parentNode.point, rand)
                nodes.append(Node(newnode, parentNode))
                pygame.draw.line(screen, cyan, parentNode.point, newnode)

                if point_circle_collision(newnode, goalPoint.point, GOAL_RADIUS):
                    currentState = 'goalFound'
                    goalNode = nodes[len(nodes) - 1]


            else:
                print("Ran out of nodes... :(")
                return;

        # handle events
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Exiting")
            if e.type == MOUSEBUTTONDOWN:
                print('mouse down')
                if currentState == 'init':
                    if initPoseSet == False:
                        nodes = []
                        if collides(e.pos) == False:
                            print('initiale point set: ' + str(e.pos))

                            initialPoint = Node(e.pos, None)
                            nodes.append(initialPoint)  # Start in the center
                            initPoseSet = True
                            pygame.draw.circle(screen, red, initialPoint.point, GOAL_RADIUS)
                    elif goalPoseSet == False:
                        print('goal point set: ' + str(e.pos))
                        if collides(e.pos) == False:
                            goalPoint = Node(e.pos, None)
                            goalPoseSet = True
                            pygame.draw.circle(screen, green, goalPoint.point, GOAL_RADIUS)
                            currentState = 'buildTree'
                else:
                    currentState = 'init'
                    initPoseSet = False
                    goalPoseSet = False
                    reset()

        pygame.display.update()
        #fpsClock.tick(10000)

if __name__ == '__main__':
    main()
