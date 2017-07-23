"""
    Team ZARJ generic task code
"""
import rospy

class ZarjTask(object):
    """ This class drives a generic task """

    def __init__(self, fc, task, checkpoints):
        self.fc = fc
        self.task_id = task
        self.checkpoints = checkpoints
        self.last_checkpoint = 0

    def begin(self, checkpoint_id):
        """ Start a particular task / checkpoint thread """
        if not checkpoint_id == self.last_checkpoint:
            self.stop()
            self.last_checkpoint = checkpoint_id
            if checkpoint_id - 1 < len(self.checkpoints):
                rospy.loginfo("Starting task {} / checkpoint {}".format(self.task_id, checkpoint_id))
                self.checkpoints[checkpoint_id - 1].start()

    def stop(self):
        """ Stop the current checkpoint thread """
        if self.last_checkpoint > 0 and \
           self.last_checkpoint - 1 < len(self.checkpoints):
            self.checkpoints[self.last_checkpoint - 1].stop()

    def status(self, msg):
        """ Handle a status message from the srcsim """
        if msg.current_checkpoint > self.last_checkpoint:
            if self.last_checkpoint > 0 and self.checkpoints[self.last_checkpoint - 1].is_done():
                self.begin(msg.current_checkpoint)
