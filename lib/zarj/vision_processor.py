"""
    Vision processor base class for the robot
"""
__all__ = ['VisionProcessor']

class VisionProcessor(object):
    """ A base class to handle vision processing """

    def __init__(self):
        self.side = 0

    def ready(self):
        """ Ready to start processing """
        return True

    def process_image(self, image, header, tag):
        """ Core of image processing """
        pass
