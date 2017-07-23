"""
    Team ZARJ task 2, checkpoint 3 code
"""
from zarjcheckpoint import ZarjCheckpoint
from zarjcomm.message import ZarjMessage

class CheckpointB3(ZarjCheckpoint):
    """ This is the class that drives checkpoint 3 """

    def __init__(self, task):
        ZarjCheckpoint.__init__(self, task, 3)
        self.started = False

    def periodic(self):
        """ Run with frequency, moving the checkpoint along """
        if not self.started:
            self.started = True
