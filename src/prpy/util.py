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

import logging, numpy, openravepy, scipy.misc, time

def create_sensor(env, args, anonymous=True):
    sensor = openravepy.RaveCreateSensor(env, args)
    if sensor is None:
        raise Exception("Creating '%s' sensor failed." % args.split()[0])

    env.Add(sensor, anonymous)
    return sensor

def GetTrajectoryIndices(traj):
    joint_values_group = traj.GetConfigurationSpecification().GetGroupFromName('joint_values')
    return numpy.array([ int(index) for index in joint_values_group.name.split()[2:] ])

def WaitForControllers(controllers, timeout=None, rate=20):
    running_controllers = set(controllers)
    start_time = time.time()
    timestep = 1.0 / rate

    while running_controllers:
        # Check for a timeout.
        now_time = time.time()
        if timeout is not None and now_time - start_time > timeout:
            return False

        # Check if the trajectory is done.
        done_controllers = set()
        for controller in running_controllers:
            if controller.IsDone():
                done_controllers.add(controller)

        running_controllers -= done_controllers
        time.sleep(timestep)

    return True

def SetCameraFromXML(viewer, xml):
    if isinstance(viewer, openravepy.Environment):
        viewer = viewer.GetViewer()

    import lxml.etree
    from StringIO import StringIO
    padded_xml = '<bogus>{0:s}</bogus>'.format(xml)
    camera_xml = lxml.etree.parse(StringIO(padded_xml))

    translation_raw = camera_xml.find('//camtrans').text
    axis_raw = camera_xml.find('//camrotationaxis').text
    focal_raw = camera_xml.find('//camfocal').text

    translation = numpy.loadtxt(StringIO(translation_raw))
    axis_angle = numpy.loadtxt(StringIO(axis_raw))
    axis_angle = axis_angle[3] * axis_angle[0:3] * (numpy.pi / 180)
    focal = float(focal_raw)

    transform = openravepy.matrixFromAxisAngle(axis_angle)
    transform[0:3, 3] = translation
    viewer.SetCamera(transform, focal)

def TakeSnapshot(viewer, path=None, show_figures=True, width=1920, height=1080, fx=640, fy=640):
    if isinstance(viewer, openravepy.Environment):
        viewer = viewer.GetViewer()

    viewer.SendCommand('SetFiguresInCamera {0:d}'.format(show_figures))
    image = viewer.GetCameraImage(width, height, viewer.GetCameraTransform(),
                                  [ fx, fy, width/2, height/2 ])

    if path is not None:
        scipy.misc.imsave(path, image)

    return image

class Recorder(object):
    MPEG = 13

    def __init__(self, env, filename, width=1920, height=1080, codec=MPEG):
        self.env = env
        self.filename = filename
        self.width = width
        self.height = height
        self.codec = codec

        self.module = openravepy.RaveCreateModule(env, 'ViewerRecorder')
        env.Add(self.module)

    def __enter__(self):
        cmd = 'Start {width:d} {height:d} 30 codec {codec:d} timing realtime filename {filename:s}'\
              '\nviewer {viewer:s}'.format(
            width = self.width,
            height = self.height,
            codec = self.codec,
            filename = self.filename,
            viewer = self.env.GetViewer().GetName()
        )
        self.module.SendCommand(cmd)

    def __exit__(self, type, value, traceback):
        self.module.SendCommand('Stop')

class Timer(object):
    def __init__(self, message):
        self.message = message
        self.start = 0 

    def __enter__(self):
        logging.info('%s started execution.', self.message)
        self.start = time.time()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()
        logging.info('%s executed in %.5f seconds.', self.message, self.get_duration())

    def stop(self):
        self.end = time.time()

    def get_duration(self):
        return self.end - self.start

