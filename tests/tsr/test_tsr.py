import numpy
from numpy import pi
from prpy.tsr import TSR
from unittest import TestCase


class TsrTest(TestCase):
    """ Test cases for prpy.tsr.Tsr. """
    def test_sample_xyzrpy(self):
        # Test zero-intervals.
        Bw = [[0.,   0.],    # X
              [1.,   1.],    # Y
              [-1., -1.],    # Z
              [0.,   0.],    # roll
              [pi,   pi],    # pitch
              [-pi, -pi]]  # yaw
        tsr = TSR(Bw=Bw)
        s = tsr.sample_xyzrpy()

        Bw = numpy.array(Bw)
        self.assertTrue(numpy.all(s >= Bw[:, 0]))
        self.assertTrue(numpy.all(s <= Bw[:, 1]))

        # Test over-wrapped angle intervals.
        Bw = [[0.,           0.],  # X
              [0.,           0.],  # Y
              [0.,           0.],  # Z
              [pi,        3.*pi],  # roll
              [pi/2.,   3*pi/2.],  # pitch
              [-3*pi/2., -pi/2.]]  # yaw
        tsr = TSR(Bw=Bw)
        s = tsr.sample_xyzrpy()

        Bw = numpy.array(Bw)
        self.assertTrue(numpy.all(s >= Bw[:, 0]))
        self.assertTrue(numpy.all(s <= Bw[:, 1]))
