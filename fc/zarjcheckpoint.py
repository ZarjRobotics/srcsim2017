"""
    Team ZARJ generic checkpoint code
"""
import threading
import rospy

class ZarjCheckpoint(threading.Thread):
    """ This is the class that drives a single checkpoint 
        You must supply a periodic() and is_done() method
        You can optionally supply a cleanup() method """

    def __init__(self, task, checkpoint):
        self.task = task
        self.checkpoint_id = checkpoint
        self.stopped = False
        threading.Thread.__init__(self)

    def run(self):
        rospy.loginfo('start task {} checkpoint {}.'.format(self.task.task_id, self.checkpoint_id))
        self.start_time = rospy.get_rostime()
        self.setup()
        while not self.stopped or not self.is_done():
            self.periodic()
            rospy.sleep(0.01)
        self.cleanup()

        rospy.loginfo('end task {} checkpoint {}.'.format(self.task.task_id, self.checkpoint_id))

    def stop(self):
        self.stopped = True

    def is_done(self):
        """ Indicate if we have finished all of our planned actions """
        return True

    def setup(self):
        """ Run once on first start """
        pass

    def cleanup(self):
        """ Run on stop """
        pass

    def periodic(self):
        """ Run with frequency, moving the checkpoint along """
        pass
