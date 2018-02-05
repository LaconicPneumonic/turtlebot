# import roslib; roslib.load_manifest('iago')
import rospy

#!/usr/bin/env python
class Particle:
    """A particle in a particle filter.
    """

    def __init__(self, x=0, y=0, theta =0, probability=0):
        """Creates a particle in a particle filter.

        Initiates the particle at the point x,y in the map
        with a probability that the object we are searching for is located there

        Args:
            x: the x coordinate of the particle
            y: the y coordinate of the particle
            probability: the probability that the object we are searching for is at this
                        particle. It must be between [0,1]
        Returns:
            A particle at the point x,y with a probability that the object we are 
            searching for is located there
        """

        

        self.x = x
        self.y = y
        self.theta = theta
        self.probability = probability

        self._check_pos()
        self._check_probability()

    def __str__(self):
        return "Particle(pos: " + str(self.get_pos()) + " | prob: " + str(self.get_probability()) + ")"

    def __repr__(self):
        return "Particle(pos: " + str(self.get_pos()) + " | prob: " + str(self.get_probability()) + ")"


    def get_pos(self):
        """Return the x,y position of this particle.

        Returns:
            A tuple (x,y, theta)
        """


        return self.x, self.y, self.theta

    def get_probability(self):
        """Return the probability that the object we are searching for is located 
            there at this point.
        """
        
        return self.probability

    def assign_probability(self, new_prob):
        """Assign the probability that the object we are searching for is located 
            there at this point.

        Throws:
            ValueError if not new_prob is not in between [0,1] 
        """

        

        self.probability = new_prob

        self._check_probability()

    def set(self, new_x, new_y, new_theta):
        """Change the x,y location of this particle

        Args:
            new_x: the new self.x of the particle
            new_y: the new self.y of the particle
        """

        
        self.x = new_x
        self.y = new_y
        self.theta = new_theta

        self._check_pos()

    def _check_pos(self):
        if ( isinstance(self.x, int) and isinstance(self.y, int)):
            if not(self.x >= 0 and self.y >= 0):
                raise ValueError("x and y must be greater than 0")
        else:
            raise TypeError("x and y must be integers")


    def _check_probability(self):
        if (isinstance(self.probability, int) or isinstance(self.probability, long) or isinstance(self.probability, float)):
            if not (0 <= self.probability <= 1):
                raise ValueError("probability must be between [0,1]")
        else:
            raise TypeError("probability must be a real number")