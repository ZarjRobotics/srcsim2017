"""
    Team ZARJ task 1, checkpoint 3 code - pitch and yaw
"""
import rospy
from zarjcheckpoint import ZarjCheckpoint

from zarjcomm.message import ZarjSatellite
from srcsim.msg import Satellite


class CheckpointA3(ZarjCheckpoint):
    """ The class that drives checkpoint 3 in task 1 """

    def __init__(self, task):
        ZarjCheckpoint.__init__(self, task, 3)
        self.sub = None
        self.satellite = None

    def _satellite_update(self, msg):
        """ Receive updates on satellite position """
        satellite = ZarjSatellite(
            round(msg.current_pitch - msg.target_pitch, 6),
            msg.pitch_correct_now,
            msg.pitch_completed,
            round(msg.current_yaw - msg.target_yaw, 6),
            msg.yaw_correct_now,
            msg.yaw_completed)
        if self.satellite is None or satellite != self.satellite:
            self.satellite = satellite
            self.task.fc.zarj_comm.push_message(self.satellite)

    def setup(self):
        """ Run once on first start """
        self.sub = rospy.Subscriber("/task1/checkpoint2/satellite",
                                    Satellite, self._satellite_update)

    def cleanup(self):
        """ cleanup at the end """
        self.sub.unregister()
