#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Michael Koval <mkoval@cs.cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import collections, logging, sys, warnings

class ColoredFormatter(logging.Formatter):
    def __init__(self, default):
        self._default_formatter = default
        self._color_table = collections.defaultdict(lambda: list())
        self._color_table[logging.CRITICAL] = [ 'red' ]
        self._color_table[logging.ERROR] = [ 'red' ]
        self._color_table[logging.WARNING] = [ 'yellow' ]
        self._color_table[logging.DEBUG] = [ 'green' ]

        # Import termcolor now to fail-fast.
        import termcolor
        self.termcolor = termcolor

    def format(self, record):
        color_options = self._color_table[record.levelno]
        message = self._default_formatter.format(record)
        return self.termcolor.colored(message, *color_options)

def initialize_logging():
    formatter = logging.Formatter('[%(levelname)s] [%(name)s:%(filename)s:%(lineno)d]:%(funcName)s: %(message)s')

    # Remove all of the existing handlers.
    base_logger = logging.getLogger()
    for handler in base_logger.handlers:
        base_logger.removeHandler(handler)

    # Add the custom handler.
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.DEBUG)
    handler.setFormatter(formatter)
    base_logger.addHandler(handler)
    base_logger.setLevel(logging.INFO)

    # Colorize logging output if the termcolor package is available.
    try:
        color_formatter = ColoredFormatter(formatter)
        handler.setFormatter(color_formatter)
    except ImportError:
        logging.warning('Install termcolor to colorize log messages.')

    # Disable spammy and ROS loggers.
    spammy_logger_names = [
        'rospy.topics',
        'openravepy.inversekinematics',
        'openravepy.databases.inversekinematics',
    ]
    for spammy_logger_name in spammy_logger_names:
        spammy_logger = logging.getLogger(spammy_logger_name)
        spammy_logger.setLevel(logging.WARNING)

    # Enable deprecation warnings, which are off by default in Python 2.7.
    warnings.simplefilter('default')

    return base_logger

def remove_ros_logger():
    logger = logging.getLogger()
    new_handlers = []

    for handler in logger.handlers:
        if type(handler).__name__ != 'RosStreamHandler':
            new_handlers.append(handler)

    logger.handlers = new_handlers

