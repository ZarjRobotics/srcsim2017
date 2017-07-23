"""
    Team ZARJ task 3, checkpoint 4 code
"""
from zarjcheckpoint import ZarjCheckpoint
import zarj

class CheckpointC4(ZarjCheckpoint):
    """ This is the class that drives checkpoint 4 """

    def __init__(self, task):
        ZarjCheckpoint.__init__(self, task, 4)

    def setup(self):
        """ Run once on first start """
        self.task.fc.zarj.zps.set_flag(zarj.zps.FLAG_HABITAT)
