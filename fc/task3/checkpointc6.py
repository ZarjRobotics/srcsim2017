"""
    Team ZARJ task 3, checkpoint 6 code
"""
import rospy
from zarjcheckpoint import ZarjCheckpoint

import zarj
from zarjcomm.message import ZarjLeak
from srcsim.msg import Leak

class CheckpointC6(ZarjCheckpoint):
    """ This is the class that drives checkpoint 6 """

    def __init__(self, task):
        ZarjCheckpoint.__init__(self, task, 6)
        self.sub = None
        self.leak = None

    def _leak_update(self, msg):
        """ Receive updates on leak position """
        leak = ZarjLeak(msg.value)
        if self.leak is None or leak != self.leak:
            self.leak = leak
            self.task.fc.zarj_comm.push_message(self.leak)


    def setup(self):
        """ Run once on first start """
        self.task.fc.zarj.zps.set_flag(zarj.zps.FLAG_HABITAT)
        self.sub = rospy.Subscriber("/task3/checkpoint5/leak", Leak,
                                    self._leak_update)

    def cleanup(self):
        """ cleanup at the end """
        self.sub.unregister()
