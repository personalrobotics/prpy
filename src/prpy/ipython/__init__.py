# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Pras Velagapudi <pkv@cs.cmu.edu>
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
"""
This module implements some custom representation types based on:
https://ipython.org/ipython-doc/dev/config/integrating.html
https://www.safaribooksonline.com/blog/2014/02/11/altering-display-existing-classes-ipython/
"""
import logging
import io
import scipy
import openravepy
from IPython import get_ipython

logger = logging.getLogger(__name__)


def EnvironmentToHTML(env):
    return "<html><body>HELLO I AM CHEESE</body></html"


def EnvironmentToPNG(env, width=640, height=480):
    """
    Takes a screenshot of the OpenRAVE environment and returns it as a PNG.

    This requires a screenshot capable viewer to be attached to the
    environment.

    @param env the environment that should be displayed
    """
    image_buffer = io.BytesIO()
    intrinsics = [width, width, width/2., height/2.]

    viewer = env.GetViewer()
    # Get the environment viewer used to generate a screenshot.
    if not viewer:
        logger.warn("Unable to generate PNG: no viewer was attached.")
        return image_buffer.getvalue()

    # Attempt to take a screenshot and compress it as a PNG.
    try:
        image = viewer.GetCameraImage(
            width, height, env.GetViewer().GetCameraTransform(), intrinsics)
        scipy.misc.imsave(image_buffer, image, format='png')
    except openravepy.openrave_exception as e:
        logger.warn("Unable to generate PNG: {:s}".format(e.message()))
    return image_buffer.getvalue()


def Initialize():
    """
    Loads IPython extensions into the current environment.
    """
    # Get IPython reference if it exists.
    ip = get_ipython()
    if not ip:
        return

    # Register a handler for the OpenRAVE HTML formatter.
    html_formatter = ip.display_formatter.formatters['text/html']
    html_formatter.for_type(openravepy.Environment, EnvironmentToHTML)

# Try to load the extensions into the current environment.
Initialize()
