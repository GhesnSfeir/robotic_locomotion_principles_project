"""
A PyrobotSimulator world. A large room with two robots and
two lights.

(c) 2005, PyroRobotics.org. Licensed under the GNU GPL.
"""

from pyrobot.simulators.pysim import *

def INIT():
    # (width, height), (offset x, offset y), scale:
    sim = TkSimulator((400, 400), (0, 400), 40)
    # x1, y1, x2, y2 in meters:
    sim.addBox(0, 0, 10, 10)
    sim.addBox(4, 4, 6, 6)
    sim.addLight(5, 8, .3)
    # port, name, x, y, th, bounding Xs, bounding Ys, color
    # (optional TK color name):
    sim.addRobot(60000, TkPioneer("RedPioneer",
                                  5, 2, 0,
                                  ((.225, .225, -.225, -.225),
                                   (.175, -.175, -.175, .175)),
                                  "#FF7700"))
    # add some sensors:
    sim.robots[0].addDevice(PioneerFrontSonars())
    return sim
