# Copyright (c) 2015, Carnegie Mellon University
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
import base64
import io
import logging
import numpy
import openravepy
import os
import scipy

logger = logging.getLogger(__name__)
_module_dir = os.path.dirname(os.path.abspath(__file__))
_render_cache = dict()


def AssimpSceneGraphToList(node, transform):
    """ Flattens an Assimp scenegraph into a list of nodes. """
    node_transform = numpy.dot(transform, node.transformation)
    node_list = [{'name': mesh.name,
                  'transform': node_transform.ravel().tolist()}
                 for mesh in node.meshes]
    for child in node.children:
        node_list += AssimpSceneGraphToList(child, node_transform)
    return node_list


def AssimpSceneToDict(filename, scene):
    """ Convert an Assimp Scene to a dictionary of properties """
    props = {
        'name': filename,
        'materials': scene.materials
    }
    props['meshes'] = {
        mesh.name: {
            'indices': base64.b64encode(
                mesh.faces.ravel().astype(numpy.uint16)),
            'normals': base64.b64encode(
                mesh.normals.ravel().astype(numpy.float32)),
            'vertices': base64.b64encode(
                mesh.vertices.ravel().astype(numpy.float32)),
            'materialindex': mesh.materialindex
        } for mesh in scene.meshes
    }
    props['entities'] = AssimpSceneGraphToList(scene.rootnode, numpy.eye(4))
    return props


def GeometryToDict(geometry):
    """ Convert an OpenRAVE Geometry to a dictionary of properties. """
    pose = geometry.GetTransformPose()
    props = {
        'quaternion': pose[[1, 2, 3, 0]],  # Reorder (w,x,y,z) to (x,y,z,w).
        'position': pose[4:],
        'color': {
            'diffuse': geometry.GetDiffuseColor(),
            'ambient': geometry.GetAmbientColor(),
            'opacity': 1.0 - geometry.GetTransparency(),
            'isTransparent': geometry.GetTransparency() > 0.0
        }
    }

    # Store information about the collision mesh.
    geometry_type = geometry.GetType()
    geometry_mesh = geometry.GetCollisionMesh()
    props['collision'] = {
        'type': str(geometry_type),
        'indices': base64.b64encode(
            geometry_mesh.indices.ravel().astype(numpy.uint16)),
        'vertices': base64.b64encode(
            geometry_mesh.vertices.ravel().astype(numpy.float32))
    }

    if geometry_type == openravepy.GeometryType.Box:
        props['collision']['extents'] = 2*geometry.GetBoxExtents()
    elif geometry_type == openravepy.GeometryType.Cylinder:
        props['collision']['height'] = geometry.GetCylinderHeight()
        props['collision']['radius'] = geometry.GetCylinderRadius()
    elif geometry_type == openravepy.GeometryType.None:
        pass
    elif geometry_type == openravepy.GeometryType.Sphere:
        props['collision']['radius'] = geometry.GetSphereRadius()
    elif geometry_type == openravepy.GeometryType.Trimesh:
        pass
    else:
        logger.warn("Found unexpected geometry type: {:d}"
                    .format(geometry_type))

    # Store information about the render mesh.
    geometry_renderfilename = geometry.GetRenderFilename()
    geometry_renderscale = geometry.GetRenderScale()
    props['render'] = {
        'filename': geometry_renderfilename,
        'scale': geometry_renderscale
    }

    if geometry_renderfilename:
        if geometry_renderfilename not in _render_cache:
            # DEBUG: fix me!
            print "LOADING {:s}".format(geometry_renderfilename)
            import pyassimp
            scene = pyassimp.load(geometry_renderfilename)
            _render_cache[geometry_renderfilename] = scene

    return props


def LinkToDict(link):
    """ Convert an OpenRAVE Link to a dictionary of properties. """
    pose = link.GetTransformPose()
    props = {
        'name': link.GetName(),
        'quaternion': pose[[1, 2, 3, 0]],  # Reorder (w,x,y,z) to (x,y,z,w).
        'position': pose[4:]
    }
    props['geometries'] = [GeometryToDict(g)
                           for g in link.GetGeometries() if g.IsVisible()]
    return props


def BodyToDict(body):
    """ Convert an OpenRAVE Body to a dictionary of properties. """
    pose = body.GetTransformPose()
    props = {
        'name': body.GetName(),
        'quaternion': pose[[1, 2, 3, 0]],  # Reorder (w,x,y,z) to (x,y,z,w).
        'position': pose[4:]
    }
    props['links'] = [LinkToDict(l)
                      for l in body.GetLinks() if l.IsVisible()]
    return props


def EnvironmentToHTML(env):
    try:
        import jinja2
    except ImportError:
        raise ImportError('HTML OpenRAVE rendering requires Jinja2.')

    try:
        import pyassimp
        assert pyassimp  # We use this import later, just test for it now.
    except ImportError:
        logger.warn("Render mesh display requires PyAssimp.")

    j2 = jinja2.Environment(
        loader=jinja2.FileSystemLoader(_module_dir)
    )

    # Get all of the geometries in this OpenRAVE environment.
    bodies = [BodyToDict(body)
              for body in env.GetBodies() if body.IsVisible()]

    # Add all the models referenced in this scene.
    models = [AssimpSceneToDict(name, model)
              for name, model in _render_cache.iteritems()]

    return j2.get_template('environment.html').render(
        bodies=bodies,
        models=models
    )


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
    # Attempt to load the IPython module.
    try:
        from IPython import get_ipython
        ip = get_ipython()
        if not ip:
            return
        # TODO: better error handling.
    except ImportError:
        raise ImportError('This feature requires IPython 1.0+')

    # Register a handler for the OpenRAVE HTML formatter.
    html_formatter = ip.display_formatter.formatters['text/html']
    html_formatter.for_type(openravepy.Environment, EnvironmentToHTML)

    # png_formatter = ip.display_formatter.formatters['image/png']
    # png_formatter.for_type(openravepy.Environment, EnvironmentToPNG)


def ClearRenderCache():
    """
    Removes all previously loaded render meshes from the cache.
    """
    _render_cache.clear()

# Try to load the extensions into the current environment.
Initialize()
