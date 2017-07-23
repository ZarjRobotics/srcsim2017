"""
    A system which, given a point in space, gives
    useful feedback on where to position other things.

"""

from copy import deepcopy
try:
    from zarj.utils import log
except:
    def log(_):
        """ Stub logging with no ros """
        pass
import math

def anchor_measure(anchor, point1, point2, offset):
    if point1 is not None and point2 is not None:
        anchor.adjusted[0] = point2[0] - point1[0]
        anchor.adjusted[1] = point2[1] - point1[1]
        anchor.adjusted[2] = point2[2] - point1[2]
        if anchor.adjusted[1] != 0:
            anchor.angle = math.degrees(math.atan(anchor.adjusted[0]/anchor.adjusted[1]))
        else:
            anchor.angle = 90.0
        return True
    else:
        log("measure anchor requires two points.")
    return False

def anchor_average(anchor, point1, point2, offset):
    if point1 is not None and point2 is not None:
        anchor.adjusted[0] = (point2[0] + point1[0]) / 2
        anchor.adjusted[1] = (point2[1] + point1[1]) / 2
        anchor.adjusted[2] = (point2[2] + point1[2]) / 2
        if anchor.adjusted[1] != 0:
            dx = point2[0] - point1[0]
            dy = point2[1] - point1[1]
            anchor.angle = math.degrees(math.atan(dx/dy))
        return True
    else:
        log("center anchor requires two points.")
    return False

def center_offset(anchor, point1, point2, offset):
    if anchor_average(anchor, point1, point2, offset):
        if offset is not None:
            anchor.adjusted[0] -= offset[0]
            anchor.adjusted[1] -= offset[1]
            anchor.adjusted[2] -= offset[2]
        return True

    return False


class Anchor(object):
    """ This gives us types and ranges for arms and hands """

    points = [
                 {
                    'name': 'identity',
                    'desc': 'This anchor will return the point chosen.',
                    'func': None,
                    'offset': [ .0, .0, 0.0]
                 },
                 {
                    'name': 'measure',
                    'desc': 'Example to show behavior',
                    'func': anchor_measure,
                    'offset': None
                 },
                 {
                    'name': 'center',
                    'desc': 'Show center and angle between two points',
                    'func': anchor_average,
                    'offset': None
                 },
                 {
                    'name': 'drop_array',
                    'desc': 'Click on outer ring of choke; position given is walking instructions for release of array',
                    'func': None,
                    'offset': [ .6, -.44, 0.0]
                 },
                 {
                    'name': 'pickup_cable',
                    'desc': 'Click on choke; position given is walking instructions for pickup up cable',
                    'func': None,
                    'offset': [ .6, -.56, 0.0]
                 },
                 {
                    'name': 'connect_cable',
                    'desc': 'Click on power jack.',
                    'func': None,
                    'offset': [ .7, -.25, 0.0]
                 },
                 {
                    'name': 'plugin_pre_start',
                    'desc': 'Way to the right of the power jack.',
                    'func': None,
                    'offset': [ 0.00, .30, -0.25]
                 },
                 {
                    'name': 'plugin_start',
                    'desc': 'Above the power jack.',
                    'func': None,
                    'offset': [ -0.02, .03, -0.20]
                 },
                 {
                    'name': 'plugin_finish',
                    'desc': 'Into the power jack.',
                    'func': None,
                    'offset': [ -0.02, .02, -0.03]
                 },
                 {
                    'name': 'task2_prep_button',
                    'desc': 'Position above button, ready to strike',
                    'func': None,
                    'offset': [-.03, -.03, -.2]
                 },
                 {
                    'name': 'rake_measure',
                    'desc': 'Use and offset from the center of two points',
                    'func': center_offset,
                    'offset': [.828, -.06, 0.0 ]
                 },
                 {
                    'name': 'vision_repair_move',
                    'desc': 'Basically the X coordinate for the vision repair move',
                    'func': None,
                    'offset': [.75, 0.0, 0.0 ]
                 },
             ]

    def __init__(self, point):
        self.name = point['name']
        self.desc = point['desc']
        self.func = point['func']
        self.offset = point['offset']
        self.adjusted = [0.0, 0.0, 0.0]
        self.angle = None

    @staticmethod
    def list():
        ret = []
        for p in Anchor.points:
            ret.append(p['name'])
        return ret

    @staticmethod
    def get(name):
        for p in Anchor.points:
            if p['name'] == name:
                return Anchor(p)
        return None

    def update(self, point1, point2):
        if self.func is not None:
            return self.func(self, point1, point2, self.offset)

        if self.offset is not None and point1 is not None:
            self.adjusted[0] = point1[0] - self.offset[0]
            self.adjusted[1] = point1[1] - self.offset[1]
            self.adjusted[2] = point1[2] - self.offset[2]
            return True

        return False
