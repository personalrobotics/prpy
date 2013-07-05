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

import logging, numpy, openravepy, threading, time

class ServoSimulator:
    def __init__(self, manip, rate, watchdog_timeout):
        self.manip = manip
        self.indices = self.manip.GetArmIndices()
        self.num_dofs = len(self.indices)
        self.q_dot = numpy.zeros(self.num_dofs)

        self.running = False
        self.watchdog = time.time()
        self.watchdog_timeout = watchdog_timeout

        self.period = 1.0 / rate
        self.timer = None
        self.mutex = threading.Lock()
        self.thread = threading.Thread(target=self.Step)
        self.thread.daemon = True
        self.thread.start()

    def SetVelocity(self, q_dot):
        q_dot_limits = self.manip.GetRobot().GetDOFVelocityLimits(self.indices)

        if not (numpy.abs(q_dot) <= q_dot_limits).all():
            raise openravepy.openrave_exception('Desired velocity exceeds limits.')

        with self.mutex:
            if (q_dot != numpy.zeros(self.num_dofs)).any():
                self.q_dot = numpy.array(q_dot, dtype='float')
                self.watchdog = time.time()
                self.running = True
            else:
                self.running = False

    def Step(self):
        while True:
            with self.mutex:
                # Copy the velocity for thread safety.
                q_dot = self.q_dot.copy()
                running = self.running

                # Stop servoing when the watchdog times out.
                now = time.time()
                if running and now - self.watchdog > self.watchdog_timeout:
                    self.q_dot = numpy.zeros(self.num_dofs)
                    self.running = False
                    logging.warning('Servo motion timed out in %.3f seconds.', now - self.watchdog)
            if running:
                with self.manip.GetRobot().GetEnv():
                    q  = self.manip.GetDOFValues()
                    q += self.period * q_dot

                    # Check joint limits.
                    with self.manip.GetRobot().CreateRobotStateSaver():
                        self.manip.SetActive()
                        q_min, q_max = self.manip.GetRobot().GetActiveDOFLimits()

                    if ((q_min <= q).all() and (q <= q_max).all()):
                        self.manip.SetDOFValues(q)
                    else:
                        self.running = False 
                        logging.warning('Servo motion hit a joint limit.')

            time.sleep(self.period)
