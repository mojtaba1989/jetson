class Error(Exception):
    """Base class for other exceptions"""
    pass


class Unstable(Error):
    """Raised when a sensor's output is unavailable"""
    pass


class LargeBuffer(Error):
    """Raised when Buffer size is large"""
    pass

