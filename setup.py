#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'prpy',
        'prpy.base',
        'prpy.planning',
        'prpy.simulation',
        'prpy.tsr',
    ],
    package_dir={'': 'src'},
)
setup(**d)
