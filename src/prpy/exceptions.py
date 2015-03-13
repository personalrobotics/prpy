class PrPyException(Exception):
    """
    Generic PrPy exception.
    """

class TrajectoryExecutionFailed(PrPyException):
    """
    Trajectory execution failed.
    """

class TrajectoryAborted(TrajectoryExecutionFailed):
    """
    Trajectory was aborted.
    """

class TrajectoryStalled(TrajectoryExecutionFailed):
    """
    Trajectory stalled.
    """

class SynchronizationException(TrajectoryExecutionFailed):
    """
    Controller synchronization failed.
    """
