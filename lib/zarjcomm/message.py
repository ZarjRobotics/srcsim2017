#!/usr/bin/env python
""" Team ZARJ Communications - Messages
    This module provides simple messages that are useful
    for sending from the field computer to the operator computer
"""

import sys
import pickle
import struct

JUST_PICKLE = 0

ZARJ_START_COMMAND = 1
ZARJ_STOP_COMMAND = 2
ZARJ_EXIT_COMMAND = 3
ZARJ_WALK_COMMAND = 4
ZARJ_MACRO_COMMAND = 5

from task0 import MACROS as MACRO_TYPES_0
from task1 import MACROS as MACRO_TYPES_1
from task2 import MACROS as MACRO_TYPES_2
from task3 import MACROS as MACRO_TYPES_3

MACRO_TYPES = MACRO_TYPES_0 + MACRO_TYPES_1 + MACRO_TYPES_2 + MACRO_TYPES_3

def demarshall_by_type(msg_type, data):
    funcs = {
        ZARJ_START_COMMAND: ZarjStartCommand.demarshall,
        ZARJ_STOP_COMMAND:  ZarjStopCommand.demarshall,
        ZARJ_EXIT_COMMAND: ZarjExitCommand.demarshall,
        ZARJ_WALK_COMMAND: ZarjWalkCommand.demarshall,
        ZARJ_MACRO_COMMAND: ZarjMacroCommand.demarshall
    }
    return funcs[msg_type](data)


def zarj_marshall_message(msg):
    if hasattr(msg, 'marshall'):
        return msg.marshall()
    header = struct.pack('!B', JUST_PICKLE)
    data = pickle.dumps(msg, pickle.HIGHEST_PROTOCOL)
    return header + data

def zarj_unmarshall_message(data):
    msg_type = struct.unpack('!B', data[:1])
    if msg_type[0] == JUST_PICKLE:
        msg = pickle.loads(data[1:])
    else:
        msg = demarshall_by_type(msg_type[0], data[1:])
    return msg


""" Messages going from the Operator Console to the Field Console
    These are all worth marshalling in a very byte efficient way,
    as they will be going the most bandwidth constrained fashion
    (as low as 60 *bits* per second).
"""

class ZarjStartCommand(object):
    def __init__(self, task, checkpoint, tell_srcsim):
        """ Start command instructs us to start a task and checkpoint """
        self.task = task
        self.checkpoint = checkpoint
        self.tell_srcsim = tell_srcsim

    def marshall(self):
        return struct.pack('!BBB?', ZARJ_START_COMMAND, self.task, self.checkpoint, self.tell_srcsim)

    @staticmethod
    def demarshall(data):
        un = struct.unpack('!BB?', data)
        return ZarjStartCommand(un[0], un[1], un[2])

class ZarjStopCommand(object):
    def __init__(self):
        """ Issue the stop all moves command """
        self.message = "Please"

    def marshall(self):
        return struct.pack('!B', ZARJ_STOP_COMMAND)

    @staticmethod
    def demarshall(data):
        return ZarjStopCommand()

class ZarjExitCommand(object):
    def __init__(self):
        """ Ask the FC to exit """
        self.message = "Please"

    def marshall(self):
        return struct.pack('!B', ZARJ_EXIT_COMMAND)

    @staticmethod
    def demarshall(data):
        return ZarjExitCommand()

class ZarjMacroCommand(object):
    def __init__(self, type, chain_okay):
        self.type = type
        self.chain_okay = chain_okay

    def marshall(self):
        if self.type in MACRO_TYPES:
            i = MACRO_TYPES.index(self.type) + 1
            return struct.pack('!BB?', ZARJ_MACRO_COMMAND, i, self.chain_okay)
        else:
            return struct.pack("!BB?B{}s".format(len(self.type)), ZARJ_MACRO_COMMAND, 0, self.chain_okay, len(self.type), self.type)

    @staticmethod
    def demarshall(data):
        un = struct.unpack('!B?', data[:2])
        if un[0] != 0:
            return ZarjMacroCommand(MACRO_TYPES[un[0] - 1], un[1])
        un = struct.unpack('!B?B', data[:3])
        un = struct.unpack("!B?B{}s".format(un[2]), data)
        return ZarjMacroCommand(un[3])

class ZarjWalkCommand(object):
    def __init__(self, distance, turn, offset, deadwalk, snapto):
        """ Instruct the robot to walk and turn """
        self.distance = distance
        self.offset = offset
        self.turn = turn
        self.deadwalk = deadwalk
        self.snapto = snapto

    def marshall(self):
        return struct.pack('!Bfff?f', ZARJ_WALK_COMMAND,
            self.distance, self.turn, self.offset, self.deadwalk, self.snapto)

    @staticmethod
    def demarshall(data):
        un = struct.unpack('!fff?f', data)
        return ZarjWalkCommand(un[0], un[1], un[2], un[3], un[4])

class ZarjIntervalCommand(object):
    def __init__(self, interval):
        """ Instruct the robot to send pictures at an interval """
        self.interval = interval

class ZarjLeanCommand(object):
    def __init__(self, angle):
        """ Instruct the robot to lean it's pelvis """
        self.angle = angle

class ZarjTurnCommand(object):
    def __init__(self, angle):
        """ Instruct the robot to turn it's pelvis """
        self.angle = angle

class ZarjNeckCommand(object):
    def __init__(self, points):
        """ Instruct the robot to position the neck """
        self.points = points

class ZarjStartWalkCommand(object):
    def __init__(self):
        """ Instruct the robot to start walking """
        self.message = "Please"

class ZarjContinueCommand(object):
    def __init__(self):
        """ Instruct the robot to keep walking """
        self.message = "Please"

class ZarjArmCommand(object):
    def __init__(self, sidename, joints):
        """ Instruct the robot to set the arm to a named configuration """
        self.sidename = sidename
        self.joints = joints

class ZarjHandCommand(object):
    def __init__(self, sidename, joints):
        """ Instruct the robot to set the hand to a named configuration """
        self.sidename = sidename
        self.joints = joints

class ZarjRequestImageCommand(object):
    def __init__(self, regular=False, stereo=True, hazard=False):
        """ Instruct the robot to send us a stereo image """
        self.regular = regular
        self.stereo = stereo
        self.hazard = hazard

class ZarjRequestLidarCommand(object):
    def __init__(self):
        """ Instruct the robot to send us a Lidar image """
        self.message = "Please"

class ZarjLocatePointCommand(object):
    def __init__(self, x, y):
        """ Ask the robot to tell us the coordinates of a given point """
        self.x = x
        self.y = y

class ZarjMovePalmCommand(object):
    def __init__(self, sidename, relative, x, y, z, yaw, pitch, roll, use_left_foot=False):
        """ Instruct the robot to move the center of the palm to the given
            x, y, z relative to the lens
        """
        self.sidename = sidename
        self.relative = relative
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.use_left_foot = use_left_foot

class ZarjGetPalmCommand(object):
    def __init__(self, sidename):
        self.sidename = sidename

class ZarjGetPalmResponse(object):
    """ A response holding position and ypr relative to left foot. """
    def __init__(self, sidename, x, y, z, yaw, pitch, roll):
        self.sidename = sidename
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

class ZarjGetArmJointsCommand(object):
    def __init__(self, sidename):
        """ Get the current joint positions for one of the hands """
        self.sidename = sidename

class ZarjMessage(object):
    def __init__(self, message):
        """ A message just contains a string """
        self.message = message

class ZarjPicture(object):
    def __init__(self, description, picture, stereo_image = False):
        """ A message containing a compressed image """
        self.description = description
        self.picture = picture
        self.stereo_image = stereo_image
        self.time = None

class ZarjLaserScan(object):
    def __init__(self, angle_min, angle_max, angle_increment, range_min, range_max, ranges):
        """ A message containing the contents of a LaserScan """
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        self.range_min = range_min
        self.range_max = range_max
        self.ranges = ranges

class ZarjStatus(object):
    def __init__(self, task, checkpoint, start_time, elapsed_time, score, harness):
        """ This is mostly a wrapper for Task, but saves Aric from installing ROS on his Mac. """
        self.task = task
        self.current_checkpoint = checkpoint
        self.start_time = start_time
        self.elapsed_time = elapsed_time
        self.score = score
        self.harness = harness

class ZarjLocatePointResponse(object):
    def __init__(self, x, y, z):
        """ Ask the robot to tell us the coordinates of a given point """
        self.x = x
        self.y = y
        self.z = z

class ZarjGetArmJointsResponse(object):
    def __init__(self, sidename, values):
        self.sidename = sidename
        self.values = values

class ZarjSatellite(object):
    def __init__(self, pitch_delta, pitch_good, pitch_done, yaw_delta, yaw_good, yaw_done):
        """ Current status of the satellite """
        self.pitch_delta = pitch_delta
        self.pitch_good  = pitch_good
        self.pitch_done  = pitch_done
        self.yaw_delta = yaw_delta
        self.yaw_good  = yaw_good
        self.yaw_done  = yaw_done

    def __eq__(self, other):
        if self.pitch_good != other.pitch_good or self.pitch_done != other.pitch_done:
            return False
        if self.yaw_good != other.yaw_good or self.yaw_done != other.yaw_done:
            return False
        if "{:1.2f}".format(self.pitch_delta) != "{:1.2f}".format(other.pitch_delta):
            return False
        if "{:1.2f}".format(self.yaw_delta) != "{:1.2f}".format(other.yaw_delta):
            return False

        return True

    def __ne__(self, other):
        return not (self == other)

class ZarjLeak(object):
    def __init__(self, value):
        """ Current status of the leak """
        self.value = value

    def __eq__(self, other):
        if self.value != other.value:
            return False
        if "{:1.2f}".format(self.value) != "{:1.2f}".format(other.value):
            return False

        return True

    def __ne__(self, other):
        return not (self == other)
