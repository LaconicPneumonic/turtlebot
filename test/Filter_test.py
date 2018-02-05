import sys
sys.path.insert(0, '../src/')

import cv2
import nose
import numpy as np

# import roslib; roslib.load_manifest('iago')
import rospy

from turtlebot import Filter, Constants

"""Testing Strategy.

Particle:
    get_path:
        returns a path of the same length as the filter
    
    sample:
        length of filter remains the same
"""

ERROR_STRING = "expected {0}, but got {1}"
MAP_WIDTH = 1000
MAP_HEIGHT = 1000



def assertEquals(expected, result):
    assert expected == result, ERROR_STRING.format(expected, result)

def test_get_path():
    class TurtleBotModel:
        def __init__(self, pose):
            self.pose = pose

        def get_pos():
            return self.pose
    test_map = Constants.OCCUPIED * np.ones((MAP_WIDTH, MAP_HEIGHT))

    cv2.circle(test_map,(MAP_WIDTH /2, MAP_HEIGHT/2), 100, Constants.OPEN, -1)

    test_filter = Filter(test_map, 50)

    test_robot = TurtleBotModel((MAP_WIDTH /2, MAP_HEIGHT/2))

    result_path = test_filter.get_path(test_robot)

    assertEquals(len(test_filter), len(result_path))


def test_sample():
    test_map = Constants.OCCUPIED * np.ones((MAP_WIDTH, MAP_HEIGHT))

    cv2.circle(test_map,(MAP_WIDTH /2, MAP_HEIGHT/2), 100, Constants.OPEN, -1)

    test_filter = Filter(test_map, 100)

    expected = len(test_filter)
    test_filter.sample()
    result = len(test_filter)

    assertEquals(expected, result)

if __name__ == '__main__':
    nose.main()
