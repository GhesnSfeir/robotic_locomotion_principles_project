# robot goes forward and then slows to a stop when it detects something

from pyrobot.brain import Brain
from math import atan2, pi

class STATE:
    GOAL_SEEK = 'GOAL_SEEK'
    WALL_FOLLOW = 'WALL_FOLLOW'
    AT_GOAL = 'AT_GOAL'

class AvoidBox(Brain):

    def __init__(self, name, engine):
        super(AvoidBox, self).__init__(name, engine)
        self.state = STATE.GOAL_SEEK
        self.goal_x = 6
        self.goal_y = 0
        self.goal_threshold = 0.1

    def computeTranslation(self, l, lf, f, rf, r):
        if f < 0.5:
            return .0
        else:
            return min([.4, f**2, lf**2, rf**2])
        
    def computeGoalSeekRot(self, goal_angle):
        goal_angle = goal_angle - 360 * int(goal_angle / (360))
        if abs(goal_angle) < 5:
            return .0
        else:
            return goal_angle / 100

    def computeRWFRot(self, l, lf, f, rf, r):
        return .3

    def determineMove(self, distance_to_goal, l, lf, f, rf, r):
        abs_goal_angle = atan2(self.goal_y - self.robot.y, self.goal_x - self.robot.x) * 180 / pi
        goal_angle = abs_goal_angle - self.robot.th
        print "Abs GA:", abs_goal_angle, "Goal angle:", goal_angle, "Robot angle:", self.robot.th
        
        if (distance_to_goal < self.goal_threshold):
            self.state = STATE.AT_GOAL
            return (.0, .0)

        forward_velocity = self.computeTranslation(l, lf, f, rf, f)
        obstacles_in_way = f < 0.9 or rf < 0.4 or lf < 0.4

        if self.state == STATE.GOAL_SEEK:
            rotation_velocity = self.computeGoalSeekRot(goal_angle)
            if obstacles_in_way:
                self.state = STATE.WALL_FOLLOW
        if self.state == STATE.WALL_FOLLOW:
            rotation_velocity = self.computeRWFRot(l, lf, f, rf, f)
            if not obstacles_in_way:
                self.state = STATE.GOAL_SEEK
        
        return (forward_velocity, rotation_velocity)
    
    def step(self):
        l = min([s.distance() for s in self.robot.range["left"]])
        lf = min([s.distance() for s in self.robot.range["left-front"]])
        f = min([s.distance() for s in self.robot.range["front"]])
        rf = min([s.distance() for s in self.robot.range["right-front"]])
        r = min([s.distance() for s in self.robot.range["right"]])
        x = self.robot.x
        y = self.robot.y

        distance_to_goal = ((x - self.goal_x) ** 2 + (y - self.goal_y) ** 2) ** 0.5

        if self.state != STATE.AT_GOAL:
            print "STATE:", self.state
            translation, rotation = self.determineMove(distance_to_goal, l, lf, f, rf, r)
            self.robot.move(translation, rotation)
            print 'Movement: Translation={0} - Rotation={1}'.format(translation, rotation)
        else:
            print "AT GOAL!!"
        
def INIT(engine):
    assert (engine.robot.requires("range-sensor") and \
            engine.robot.requires("continuous-movement"))
    return AvoidBox('AvoidBox', engine)

