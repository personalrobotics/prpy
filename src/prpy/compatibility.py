from distutils.version import LooseVersion
import numpy

# handle older versions of numpy without norm axis argument
if LooseVersion(numpy.__version__) < LooseVersion('1.8.0'):
    oldnorm = numpy.linalg.norm
    def newnorm(x, ord=None, axis=None):
        if axis is None:
            return oldnorm(x, ord=ord)
        elif isinstance(axis, (int,long)):
            return numpy.apply_along_axis(lambda x: oldnorm(x,ord=ord), axis, x)
        else:
            raise NotImplementedError('older numpy without norm axis')
    numpy.linalg.norm = newnorm
