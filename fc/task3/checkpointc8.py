"""
    Team ZARJ task 3, checkpoint 8 code
"""
from zarjcheckpoint import ZarjCheckpoint
import zarj

class CheckpointC8(ZarjCheckpoint):
    """ This is the class that drives checkpoint 8 """

    def __init__(self, task):
        ZarjCheckpoint.__init__(self, task, 8)

    def setup(self):
        """ Run once on first start """
        self.task.fc.zarj.zps.set_flag(zarj.zps.FLAG_HABITAT)
