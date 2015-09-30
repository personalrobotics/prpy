class PrPyException(Exception):
    """
    Generic PrPy exception.
    """


class TrajectoryException(PrPyException):
    """
    Trajectory failed to execute.
    """


class TrajectoryNotExecutable(TrajectoryException):
    """
    Trajectory could not begin execution.

    This exception typically indicates that some precondition of trajectory
    execution was violated, such as the robot starting at a different
    configuration, the trajectory not being in the correct format.

    This exception indicates that the trajectory was not even attempted due to
    one of these conditions.
    """


class TrajectoryAborted(TrajectoryException):
    """
    Trajectory was aborted.
    """


class TrajectoryStalled(TrajectoryException):
    """
    Trajectory stalled.
    """


class SynchronizationException(PrPyException):
    """
    Controller synchronization failed.
    """


class SerializationException(PrPyException):
    """
    Serialization failed.
    """


class UnsupportedTypeSerializationException(SerializationException):
    """
    Serialization failed due to an unknown type.
    """
    def __init__(self, value):
        self.value = value
        self.type = type(self.value)

        super(UnsupportedTypeSerializationException, self).__init__(
            'Serializing type "{:s}.{:s}" is not supported.'.format(
                self.type.__module__, self.type.__name__))


class UnsupportedTypeDeserializationException(SerializationException):
    """
    Deserialization failed due to an unknown type.
    """
    def __init__(self, type_name):
        self.type_name = type_name

        super(UnsupportedTypeDeserializationException, self).__init__(
            'Deserializing type "{:s}" is not supported.'.format(type_name))
