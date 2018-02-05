import math
import thread
import threading

import numpy as np
import cv2

from turtlebot import Filter, Particle, Constants

# import roslib; roslib.load_manifest('iago')
import rospy

import actionlib
from nav_msgs.msg import OccupancyGrid, Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *


class TurtleBotModel:
    def __init__(self):
        self.view_angle = math.pi * 2.0 / 5.0
        self.view_depth = 20
    def get_pos(self):
        return 20

    def get_radius(self):
        return 10

    def get_view_shape(self,x, y, theta):

        theta *= 180.0 / math.pi

        return (x,y),(self.view_depth, self.view_depth), (theta - 180) % 360, 45, 315

THRESHOLD = 4000

def main():

    robot = TurtleBotModel()

    rospy.init_node("robot")

    test_filter = Filter(10, robot)

    test_filter.sample(robot)

    path = test_filter.get_path(robot)

    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Wait for the action server to come up")

    # Allow up to 5 seconds for the action server to come up
    move_base.wait_for_server(rospy.Duration(5))

    resampled = False
    # print path
    try:
        while len(path) != 0:
            pose = min(path, key=lambda p: test_filter.position_tracer.value_of_view(p.position.x, p.position.y))
            path.remove(pose)

            rospy.loginfo("Pursuing point")
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = pose

            # Start moving
            move_base.send_goal(goal)

            if (test_filter.position_tracer.value_of_view(pose.position.x, pose.position.y)) > THRESHOLD:
                if not resampled:
                    resampled = True
                    test_filter.sample(robot)
                    path += test_filter.get_path(robot)
                    rospy.loginfo("Resampled!")
                else:
                    # If the filter just resampled and it the minimum is above the threshold, then we have
                    # completed our search
                    break
            else:
                # If the value of the view is not greater than the threshold after one resampling, then
                # we return resampled to false.
                resampled = False

            print "value of view:" , test_filter.position_tracer.value_of_view(pose.position.x, pose.position.y)

            # Allow TurtleBot up to 60 seconds to complete task
            success = move_base.wait_for_result(rospy.Duration(45)) 

            state = move_base.get_state()
            
            if success and state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Reached point")
            else:
                rospy.loginfo("Rejected point")
                move_base.cancel_goal()

    except KeyboardInterrupt:
        print "KeyboardInterrupt!"
        move_base.cancel_goal()


    rospy.loginfo("Finished!")
    test_filter.position_tracer.stop()
    test_filter.position_tracer.join()
    cv2.destroyAllWindows()


def pretty(matrix):
    r, c = matrix.shape

    to_display = np.zeros((r, c, 3))

    for i in range(r):
        for j in range(c):
            
            data = matrix[i,j]     

            if data == Constants.OCCUPIED:
                to_display[i,j] = (255,255,255)

            elif data == Constants.OPEN:
                to_display[i,j] = (0  ,  0,  0)

            elif data == Constants.UNEX:
                to_display[i,j] = (255,  0,  0)

            elif data == Constants.EXPL:
                to_display[i,j] = (0  ,255,  0)

    return to_display

if __name__ == '__main__':
    main()  