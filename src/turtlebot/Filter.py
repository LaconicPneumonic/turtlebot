#!/usr/bin/env python3
from Particle import Particle

import math
import random
import thread
import threading
import tf
import numpy as np
import cv2

# import roslib; roslib.load_manifest('iago')
import rospy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped as PCS


class Constants:
    # We love magic numbers!
    OCCUPIED = 100
    OPEN = 0
    UNEX = -1
    EXPL = 50


class Filter(list):
    """A Filter used to randomply sample a map

    This filter will contain n particles which will be used to navigate
    the turtlebot
    """

    def __init__(self, num_particles, robot_model):
        """Creates the filter with all particles
        """

        temp_internal = list()
        self.N = num_particles
        for i in xrange(num_particles):
            temp_internal.append(Particle())

        super(Filter, self).__init__(temp_internal)

        self.map_lock = threading.Lock()
        sub = rospy.Subscriber("map", OccupancyGrid, self._state_callback)

        
        rospy.sleep(.1)

        self.position_tracer = self.robot_tracker(robot_model)
        self.position_tracer.start()

    def _state_callback(self, data):
        """Parses data from nav_msgs/OccupancyGrid

        Used to init and update Map
        """

        with self.map_lock:
            self.origin = (data.info.origin.position.x, data.info.origin.position.y)
            self.resolution = data.info.resolution
            self.map = np.asarray(data.data, dtype=np.int8).reshape(data.info.height, data.info.width).T
            self.rows, self.cols = self.map.shape
            print self.map

    def _map_to_real(self, x, y):
        """Translate map coordinates to real world coordinates

        Returns:
            (x, y, theta)
        """
        real_x = (x * self.resolution) + self.origin[0]
        real_y = (y * self.resolution) + self.origin[1]

        return real_x, real_y

    def _real_to_map(self, real_x, real_y):
        """Translate real world coordinates to map coordinates
        """

        x = (real_x - self.origin[0]) / self.resolution
        y = (real_y - self.origin[1]) / self.resolution

        return x, y

    def _valid_sample(self, point, robot_model):
        """Returns true if outside of robot_model.radius
        """


        bounding_points = (point[0] - robot_model.get_radius(),point[0] + robot_model.get_radius(), point[1] - robot_model.get_radius(), point[1] + robot_model.get_radius())

        if not ((0,0) < bounding_points[:2] < self.map.shape and (0,0) < bounding_points[2:] < self.map.shape):
            return False

        map_segment = self.map[bounding_points[0]:bounding_points[1], bounding_points[2]:bounding_points[3]]

        it = np.nditer(map_segment, flags=['multi_index'])
        while not it.finished:
            if it[0] == Constants.OCCUPIED:
                dist_from_origin = math.sqrt((it.multi_index[0] - robot_model.get_radius()) ** 2 + (it.multi_index[1] - robot_model.get_radius()) ** 2)
                if dist_from_origin < robot_model.get_radius():
                    return False
            it.iternext()


        return True
    def sample(self, robot_model):
        """Samples and sets self.N particles in the map
        """

        with self.map_lock:
            random.seed()

            n = 0

            while n < len(self):
                rand = (int(random.random() * self.rows), int(random.random() * self.cols), random.random() * 2 * math.pi)

                if self.map[rand[0], rand[1]] == Constants.OPEN:
                    if self._valid_sample(rand, robot_model):
                        self[n].set(rand[0], rand[1], rand[2])
                        self[n].assign_probability(1.0/ len(self))
                        n += 1

    def observe(self, robot_model):
        """Adjust the particles based off of the current model of the robot

        This function will take in the robot model, pose and other parameters,
        and adjust the particles based off of what the robot has seen.

        Args:
            robot_model: an object containing what the robot has seen given its state
        """
        pass

    def get_path(self, robot_model):
        """Create and return a path based of the current pose of the robot

        This function will take in the robots current pose and return a
        list of poses as a path that will allow the robot to view all points in the map.

        Args:
            robot_model: an object containing what the robot has seen given its state

        Returns:
            A list of poses that will route the robot to every point in the map.
        """ 

        pose_list = list()

        for particle in self:
            poseStamped_to_add = PoseStamped()
            pose_to_add = Pose()

            real_position = self._map_to_real(*particle.get_pos()[:2])
            
            pose_to_add.position.x, pose_to_add.position.y = real_position

            # Always yaw
            pose_to_add.orientation.x = 0
            pose_to_add.orientation.y = 0
            pose_to_add.orientation.z = math.sin(particle.get_pos()[2] * 0.5)
            pose_to_add.orientation.w = math.cos(particle.get_pos()[2] * 0.5)


            pose_list.append(pose_to_add)

        return pose_list

    def robot_tracker(self, robot_model):
        robot_listener_thread = updateMap(robot_model, self.map, self.resolution, self.origin, self.map_lock)

        return robot_listener_thread


class updateMap (threading.Thread):

    SEEN = (255,255,255)

    def __init__(self, robot_model, ros_map, resolution, origin, map_lock):
        threading.Thread.__init__(self)
        
        self.rows, self.cols = ros_map.shape
        self.map = np.zeros((self.cols, self.rows,3), np.uint8)
        self.map_lock = map_lock
        self.resolution = resolution
        self.origin = origin
        self.robot_model = robot_model
        self.x = self.y = self.theta = 0

        self.shutdown_flag = threading.Event()
        

    def _state_callback(self,data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        


        t1 = 2.0 * data.pose.pose.orientation.z * data.pose.pose.orientation.w
        t2 = 1.0 - 2.0 * (data.pose.pose.orientation.z * data.pose.pose.orientation.z)
        self.theta = math.atan2(t1, t2)

        self._observe_on_map()

    def _observe_on_map(self):
        with self.map_lock:
            x, y = self._real_to_map(self.x, self.y)

            center, axes, angle, startAngle, endAngle = self.robot_model.get_view_shape(x, y, self.theta)

            cv2.ellipse(self.map, (x, y), axes, int(angle), startAngle, endAngle, updateMap.SEEN,-1)

            # cv2.imshow("test", np.swapaxes(self.map,0,1))        
            # cv2.waitKey(1)

    def _real_to_map(self, real_x, real_y):
        """Translate real world coordinates to map coordinates
        """

        x = (real_x - self.origin[0]) / self.resolution
        y = (real_y - self.origin[1]) / self.resolution

        return int(x), int(y)

    def check_valid(self, real_x, real_y):
        x, y = self._real_to_map(real_x, real_y)
        return tuple(self.map[x,y]) != updateMap.SEEN

    def value_of_view(self, real_x, real_y):
        """
        Model the view of the robot as a box. Return the number of occupied spaces in
        the box.
        """
        x, y = self._real_to_map(real_x, real_y)

        r = self.robot_model.view_depth

        lower_x = x - r
        upper_x = x + r
        lower_y = y - r
        upper_y = y + r

        if (lower_x < 0):
            lower_x = 0
        if (upper_x >= self.rows):
            upper_x = self.rows - 1
        if (lower_y < 0):
            lower_y = 0
        if (upper_y >= self.cols):
            upper_y = self.cols -1
        sub_matrix = self.map[lower_x: upper_x, lower_y: upper_y, :].flatten()

        # It would be better to choose poses that are farther away because you
        # are likely to see more
        distance_from_self = math.sqrt((self.x - x) ** 2  + (self.y - y) ** 2)
        ret = sub_matrix[np.where(sub_matrix == 255)].shape[0] / distance_from_self

        temp = self.map.copy()

        cv2.rectangle(temp, (lower_x, lower_y), (upper_x, upper_y), (255,0,0))

        # cv2.imshow("preview", np.swapaxes(temp,0,1))
        # cv2.waitKey(10)
        return ret


    def get_map(self):
        return self.map

    def run(self):

        rate = rospy.Rate(10.0)

        listener = tf.TransformListener()

        while not self.shutdown_flag.is_set():

            # self.sub = rospy.Subscriber("amcl_pose", PCS, self._state_callback)
            # rospy.sleep(.1)
            # 
            
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self.x = trans[0]
            self.y = trans[1]

            t1 = 2.0 * rot[2] * rot[3]
            t2 = 1.0 - 2.0 * (rot[2] *rot[2])
            self.theta = math.atan2(t1, t2)

            self._observe_on_map()

            rate.sleep()

    def stop(self):
        self.shutdown_flag.set()