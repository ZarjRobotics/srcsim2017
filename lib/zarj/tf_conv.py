""" Convenience functions for transforming from one type to another and to do rotations and translations 
    We use the following abbreviations
      q is a quaternion represent as an array of four values.
      msg_q is a value of type geometry_msgs.msg.Quaternion
      v is a vector as an array of three values
      msg_v is a vector value as a geometry_msgs.msg.Vector3.
      
    Generally the idea is to do the math on the array versions of the values.
"""
import traceback
from math import degrees
from math import radians
from math import acos
from math import sqrt
from math import atan2
from math import cos
from math import sin
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import numpy

__ALL__ = ['msg_q_to_q', 'msg_v_to_v', 'q_to_msg_q', 'v_to_msg_v', 'add_v', 'add_msg_v', 'sub_v',
           'normalize_q', 'conjugate_q', 'negative_v', 'mult_q', 'compute_rot_angle', 'rotate_v', 'unrotate_v',
           'yaw_pitch_roll_to_q', 'q_to_yaw_pitch_roll', 'q_to_dir_and_roll',
           'dir_and_roll_to_yaw_pitch_roll', 'dir_and_roll_to_q',
           'fmt_q_as_dir_and_roll', 'fmt_q_as_ypr', 'fmt']


def msg_q_to_q(q):
    return [q.x, q.y, q.z, q.w]


def msg_v_to_v(v):
    return [v.x, v.y, v.z]


def q_to_msg_q(v):
    return Quaternion(v[0], v[1], v[2], v[3])


def v_to_msg_v(v):
    return Vector3(v[0], v[1], v[2])


def add_v(v1, v2):
    return [v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]]


def add_msg_v(msg_v1, msg_v2):
    return Vector3(msg_v1.x + msg_v2.x, msg_v1.y + msg_v2.y, msg_v1.z + msg_v2.z)


def sub_v(v1, v2):
    return [v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]]


def normalize_q(q):
    d = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    return [q[0] / d, q[1] / d, q[2] / d, q[3] / d]


def conjugate_q(q):
    """ Conjugate a quaternion represented as a vector """
    return [-q[0], -q[1], -q[2], q[3]]


def negative_v(v):
    return [-v[0], -v[1], -v[2]]


def mult_q(q1, q2):
    return quaternion_multiply(q1, q2)


def compute_rot_angle(msg_q):
    """ Compute the angle in degrees of rotation in a quaternion that represents a rotation around the z-axis. """
    if msg_q.z >= 0:
        rot_sign = 1
    else:
        rot_sign = -1
    return degrees(2 * acos(msg_q.w) * rot_sign)


def rotate_v(v, q):
    """ Rotate vector by a quaternion with quaternion represented as an array. """
    conj = conjugate_q(q)
    v_as_q = [v[0], v[1], v[2], 0]
    m = mult_q(q, v_as_q)
    return mult_q(m, conj)[:3]


def unrotate_v(v, q):
    """Like *rotate_v* but rotates the other way. """
    conj = conjugate_q(q)
    v_as_q = [v[0], v[1], v[2], 0]
    m = mult_q(conj, v_as_q)
    return mult_q(m, q)[:3]


def yaw_pitch_roll_to_q(ypr):
    """ 
    Creates quaternion representation of roll, pitch, and roll given as degrees. 
    Rotations around an axis are defined by sticking your thumb in the positive direction
    of the axis and then the motion of your fingers closing from an open state to a fist
    represents positive angle rotation. Based on this adhoc thumb rule, we make the following
    definitions.
    ypr[0] = yaw - Rotation around z axis. Note, unlike in aircraft or boats, the z-axis is pointing up
          which means the robot will turn to the left when doing positive yaw.
    ypr[1] = pitch - Rotation around the relocated y axis after applying yaw. The world y-axis in
            Gazebo goes from right to left, so pitch is bending forward in the gazebo reference frame.
    ypr[2] = roll - Rotation around the the relocated x axis after applying yaw and then pitch. Since the
           Gazebo x-axis points forward at the start, a roll is bending to the right.
    """
    return quaternion_from_euler(radians(ypr[0]), radians(ypr[1]), radians(ypr[2]), 'rzyx')


def q_to_yaw_pitch_roll(q):
    """ This inverts the yaw_pitch_roll_to_q function """
    e = euler_from_quaternion(q, 'rzyx')
    return [degrees(e[0]), degrees(e[1]), degrees(e[2])]


def q_to_dir_and_roll(q):
    """
    Takes an orientation and returns the unit direction it is pointing at and the roll that has been applied
    to that direction. The roll is measured in degrees. This function has a human 2-d perspective bias
    where a unit direction vector is achieved by applying yaw and pitch only, and then the roll is 
    applied after that.
    """
    dir_u = rotate_v([1, 0, 0], q)
    ypr = q_to_yaw_pitch_roll(q)
    return dir_u, ypr[2]


def dir_and_roll_to_yaw_pitch_roll(dir_u, roll):
    yaw = degrees(atan2(dir_u[1], dir_u[0]))
    d = sqrt(dir_u[0] * dir_u[0] + dir_u[1] * dir_u[1])
    pitch = degrees(atan2(dir_u[2], d))
    return [yaw, pitch, roll]


def dir_and_roll_to_q(dir_u, roll):
    """ Inverse of q_to_dir_and_roll """
    ypr = dir_and_roll_to_yaw_pitch_roll(dir_u, roll)
    return yaw_pitch_roll_to_q(ypr)


def compute_distance_to_wall(a, r1, r2):
    """ Given a sighting that determines a distance of r1 to the wall, and then a rotation by angle *a*
        to the left and a sighting of distance r2, returns the angle that we are currently rotated by
        from the perpendicular to the wall and the current distance to the wall as a pair tuple. Angle
        is given in degrees before the rotation from r1 to r2. Rotating to the right by this angle
        should cause us to face the wall directly.
    """
    try:
        if r1 < r2:
            r = r1/r2
            i = 1.0
        elif r1 > r2:
            r = r2/r1
            i = -1.0
        else:
            return 0, r2
        d = radians(a)
        c = cos(d)
        s = sin(d)
        dt = sqrt(1 - c * c * r * r)
        x = c * c * r + s * dt
        return degrees(acos(x)) * i, r2 * x
    except ValueError:
        traceback.print_exc()
        return None, None


def fmt_q_as_dir_and_roll(q):
    dir_u, roll = q_to_dir_and_roll(q)
    return 'dir={},roll={}'.format(fmt(dir_u), fmt(roll))


def fmt_q_as_ypr(q):
    ypr = q_to_yaw_pitch_roll(q)
    return '(yaw={},pitch={},roll={})'.format(fmt(ypr[0]), fmt(ypr[1]), fmt(ypr[2]))


def fmt(obj, nest_level=0):
    """ Format any common object """
    if nest_level > 10:
        return ""
    if isinstance(obj, float):
        return "{0:.3f}".format(obj)
    if isinstance(obj, list):
        return "(" + ",".join(map(lambda x: fmt(x, nest_level + 1), obj)) + ")"
    if isinstance(obj, (numpy.ndarray, numpy.generic)):
        return fmt(obj.tolist(), nest_level + 1)
    if isinstance(obj, dict):
        pairs = map(lambda x, y: "(" + fmt(x) + "," + fmt(y, nest_level + 1) + ")", obj.items())
        return fmt(pairs)
    if isinstance(obj, Vector3):
        return "({},{},{})".format(fmt(obj.x), fmt(obj.y), fmt(obj.z))
    if isinstance(obj, Quaternion):
        return "({},{},{},{})".format(fmt(obj.x), fmt(obj.y), fmt(obj.z), fmt(obj.w))
    # print " obj " + str(obj) + " is of type " + str(type(obj))
    return str(obj)


if __name__ == '__main__':
    print "90 degree yaw as quaternion"
    y90 = yaw_pitch_roll_to_q([90, 0, 0])
    print y90

    print "90 degree pitch as quaternion"
    p90 = yaw_pitch_roll_to_q([0, 90, 0])
    print p90

    print "90 degree roll as quaternion"
    r90 = yaw_pitch_roll_to_q([0, 0, 90])
    print r90

    print "90 degree yaw followed by 90 degree pitch as quaternion"
    y_p_90 = yaw_pitch_roll_to_q([90, 90, 0])
    print y_p_90

    print "Same as before but take the product of the yaw and pitch quaternions"
    print mult_q(y90, p90)  # y90 followed by p90 on relocated y-axis (equivalent to reversion product).

    print "Undo back to 90 degree yaw and 90 degree pitch from quaternion"
    print q_to_yaw_pitch_roll(y_p_90)

    print "Quaternion of 30 degree yaw and 30 degree pitch"
    q_yaw_30 = yaw_pitch_roll_to_q([30, 0, 0])
    q_pitch_30 = yaw_pitch_roll_to_q([0, 30, 0])
    q_30_30 = mult_q(q_yaw_30, q_pitch_30)  # yaw_pitch_roll_to_q([30,0,0])
    print q_30_30
    print q_to_yaw_pitch_roll(q_30_30)

    print "Direction of 30 degree yaw and 30 degree pitch"
    dir30_u, roll_30 = q_to_dir_and_roll(q_30_30)
    print q_to_dir_and_roll(q_pitch_30)
    print dir30_u

    print "Yaw, pitch, roll of direction and roll created"
    print dir_and_roll_to_yaw_pitch_roll(dir30_u, roll_30)

    print "Quaternion when undoing conversion to direction and roll"
    print dir_and_roll_to_q(dir30_u, roll_30)

    print "Computation of distance to wall should give 30 degrees rotation and 3/2 distance"
    # First sighting, 1 unit from wall. Second sighting after rotating to the left by 30
    # degrees, and distance is square root of 3.
    print compute_distance_to_wall(30, 1, sqrt(3.0))
