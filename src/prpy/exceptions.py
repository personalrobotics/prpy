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
