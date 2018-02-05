import sys
sys.path.insert(0, '../src/')

import nose

# import roslib; roslib.load_manifest('iago')
import rospy

from turtlebot.Particle import Particle

"""Testing Strategy.

Particle:
    
    Creator test

    returns correct position

    returns probability

    rmoves the poin 
"""

ERROR_STRING = "expected {0}, but got {1}"

def assertEquals(expected, result):
    assert expected == result, ERROR_STRING.format(expected, result)

def test_create():

    try:
        test_particle = Particle(0,0, 3.2, .5)
        assert True
    except:
        assert False

def test_return_position():
    expected = (3,0, 3.2)
    test_particle = Particle(3,0,3.2,.5)
    result = test_particle.get_pos()
    assertEquals(expected, result)

def test_return_probability():
    expected = .5
    test_particle = Particle(3,0,3.2,.5)
    result = test_particle.get_probability()

    assertEquals(expected, result)    

def test_probability_too_large():

    try:
        test_particle = Particle(0,0,3.2,1.5)
        assert False
    except ValueError:
        assert True


def test_set():
    expected = (30,20,3)
    test_particle = Particle(0,2,3.2,.5)

    test_particle.set(30,20,3);
    result = test_particle.get_pos()

    assertEquals(expected, result)


def test_string():
    test_particle = Particle(0,0,3.2, .745)

    print(test_particle)


if __name__ == '__main__':
    nose.main()
