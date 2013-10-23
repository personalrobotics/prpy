class PrPyException(Exception):
    """
    Generic PrPy exception.
    """

class TrajectoryAborted(PrPyException):
    """
    Trajectory was aborted.
    """

class TrajectoryStalled(PrPyException):
    """
    Trajectory stalled.
    """

class SynchronizationException(PrPyException):
    """
    Controller synchronization failed.
    """
