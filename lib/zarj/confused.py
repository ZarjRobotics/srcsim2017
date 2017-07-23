"""
    Zarj Confused.  Tell Zarj what to do.  Please
"""

__all__ = ['ZarjConfused']

class ZarjConfused(Exception):
    """ Something is off... """

    def __init__(self, message):
        super(ZarjConfused, self).__init__()
        self.message = message

    def __str__(self):
        """ String Operator """
        return 'ZarjConfused: ' + self.message

    def __repr__(self):
        """ repr """
        return self.message
