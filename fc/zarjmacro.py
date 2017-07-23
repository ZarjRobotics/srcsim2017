"""
    Team ZARJ macro base class
"""
class ZarjMacro(object):
    """ This class drives macros """

    def __init__(self, fc, chain_okay):
        self.done = False
        self.stop = False
        self.fc = fc
        self.chain_okay = chain_okay

    def start(self, previous=None):
        """ Start the macro """
        pass

    @property
    def is_done(self):
        """ return if done """
        return self.done

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        pass
