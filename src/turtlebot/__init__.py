from Particle import Particle
from Filter import Filter

# import roslib; roslib.load_manifest('iago')

class Constants:
    # We love magic numbers!
    OCCUPIED = 100
    OPEN = 0
    UNEX = -1
    EXPL = 50