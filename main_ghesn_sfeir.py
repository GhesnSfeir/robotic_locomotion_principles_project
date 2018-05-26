# robot goes forward and then slows to a stop when it detects something

from pyrobot.brain import Brain
from enum import Enum
from math import atan2, pi

class STATE(Enum):
    # OBSTACLE_FRONT = 'OBSTACLE_FRONT'
    # OBSTACLE_FRONT_LEFT = 'OBSTACLE_FRONT_LEFT'
    # OBSTACLE_LEFT = 'OBSTACLE_LEFT'
    # OBSTACLE_FRONT_RIGHT = 'OBSTACLE_FRONT_RIGHT'
    # OBSTACLE_RIGHT = 'OBSTACLE_RIGHT'
    # NO_OBSTACLE = 'NO_OBSTACLE'
    GOAL_SEEK = 0
    WALL_FOLLOW = 1
    AT_GOAL = 2

class ACTION(Enum):
    MOVE_BACKWARDS = (-1.0, -1.0, 0.00)
    TURN_LEFT = (-1.195, 1.195, 0.50)
    TURN_RIGHT = (1.195, -1.195, 0.50)
    TURN_BACKWARDS = (1.195, -1.195, 1.00)
    SLIGHTLY_TURN_LEFT = (0.8, 1.8, 0.00)
    SLIGHTLY_TURN_RIGHT = (1.8, 0.8, 0.00)
    MOVE_FORWARD = (1.8, 1.8, 0.00)
    MOVE_FORWARD_LONG = (1.8, 1.8, 0.20)
    STOP = (0, 0, 0.00)

class Avoid(Brain):

    def __init__(self, name, engine):
        super(Avoid, self).__init__(name, engine)
        self.state = STATE.GOAL_SEEK
        self.goal_x = 3
        self.goal_y = 0
        self.goal_threshold = 0.1

    # Give the front two sensors, decide the next move
    # def determineMove(self, front, left, right):
    #     if front < 0.5: 
    #         #print "obstacle ahead, hard turn"
    #         return(0, .3)
    #     elif left < 0.8:
    #         #print "object detected on left, slow turn"
    #         return(0.1, -.3)
    #     elif right < 0.8: 
    #         #print "object detected on right, slow turn" 
    #         return(0.1, .3)
    #     else:
    #         #print "clear"
    #         return(0.5, 0.0)

    def computeTranslation(self, l, lf, f, rf, r):
        if f < 0.5:
            return .0
        else:
            return min(.5, f - .5)
        
    def computeGoalSeekRot(self, goal_angle):
        if abs(goal_angle) < pi/10:
            return .0
        else:
            return goal_angle * 5

    def computeRWFRot(self, l, lf, f, rf, r):
        return .1

    def determineMove(self, distance_to_goal, l, lf, f, rf, r):
        goal_angle = atan2(self.goal_y - self.robot.y, self.goal_x - self.robot.x) - self.robot.th
        print "Goal angle:", goal_angle, "Robot angle:", self.robot.th
        
        if (distance_to_goal < self.goal_threshold):
            self.state = STATE.AT_GOAL
            return (.0, .0)

        forward_velocity = self.computeTranslation(l, lf, f, rf, f)
        obstacles_in_way = f < 0.9

        if self.state == STATE.GOAL_SEEK:
            rotation_velocity = self.computeGoalSeekRot(goal_angle)
            if obstacles_in_way:
                self.state = STATE.WALL_FOLLOW
        if self.state == STATE.WALL_FOLLOW:
            rotation_velocity = self.computeRWFRot(l, lf, f, rf, f)
            if not obstacles_in_way:
                self.state = STATE.GOAL_SEEK
        
        return (forward_velocity, rotation_velocity)

    # def getState(self, l, lf, f, rf, r, x, y, th):
    #     distance_to_goal = ((x - self.goal_x) ** 2 + (x - self.goal_x) ** 2) ** 0.5
    #     goal_reached = distance_to_goal < self.goal_threshold

    #     if goal_reached:
    #         # GOAL REACHED!
    #         return STATE.AT_GOAL
        
    #     can_go_forward = f > 0.5
    #     is_on_goal_line = -0.01 < y < 0.01
    #     is_on_oriented_to_goal = -0.01 < th < 0.01

    #     if can_go_forward and is_on_goal_line and is_on_oriented_to_goal:
    #         return STATE.GOAL_SEEK
    #     else:
    #         return STATE.WALL_FOLLOW
    
    # def step(self):
    #     lf = min([s.distance() for s in self.robot.range["left-front"]])
    #     f = min([s.distance() for s in self.robot.range["front"]])
    #     rf = min([s.distance() for s in self.robot.range["right-front"]])

    #     translation, rotate = self.determineMove(f, lf, rf)
    #     self.robot.move(translation, rotate)

    #     print self.robot.x, self.robot.y, self.robot.th, self.state
    
    def step(self):
        l = min([s.distance() for s in self.robot.range["left"]])
        lf = min([s.distance() for s in self.robot.range["left-front"]])
        f = min([s.distance() for s in self.robot.range["front"]])
        rf = min([s.distance() for s in self.robot.range["right-front"]])
        r = min([s.distance() for s in self.robot.range["right"]])
        x = self.robot.x
        y = self.robot.y
        # th = self.robot.th
        # state = self.getState(l, lf, f, rf, r, x, y, th)

        distance_to_goal = ((x - self.goal_x) ** 2 + (y - self.goal_y) ** 2) ** 0.5
        # goal_reached = distance_to_goal < self.goal_threshold

        if self.state != STATE.AT_GOAL:
            translation, rotation = self.determineMove(distance_to_goal, l, lf, f, rf, r)
            self.robot.move(translation, rotation)
        else:
            print "AT GOAL!!"
        
        # print self.robot.x, self.robot.y, self.robot.th, self.state
        print "STATE:", self.state
        print 'Translation={0} - Rotation={1}'.format(translation, rotation)

def INIT(engine):
    assert (engine.robot.requires("range-sensor") and \
            engine.robot.requires("continuous-movement"))
    return Avoid('Avoid', engine)

