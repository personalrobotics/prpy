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

import openravepy
import bind

class KinBodyDatabase(object):
    DATABASE_NAME = 'prpy'
    DATABASE_TABLE = 'ordata'

    def __init__(self):
        try:
            import pymongo
        except ImportError:
            raise Exception('KinBodyDatabase requires MongoDB and pymongo to be installed.')

        self.client = pymongo.MongoClient()
        self.database = self.client[DATABASE_NAME]
        self.table = self.database[DATABASE_TABLE]

    def bind(self):
        openravepy.Environment.ReadKinBodyDB = self.ReadKinBodyDB

    def ReadKinBodyDB(self, env, name):
        matches = self.table.find({ 'name': name })

        # Check for missing and duplicate entries.
        num_matches = matches.count()
        if num_matches == 0:
            raise KeyError('There is no object named "{0:s}".'.format(name))
        elif num_matches > 1:
            raise KeyError('There are {0:d} objects named "{1:s}".'.format(
                num_matches, name))

        # Construct the KinBody.
        metadata = matches[0]
        kinbody = env.ReadKinBodyXMLData(metadata['openrave_xml'])
        kinbody.SetName(metadata['name'])

        # Attach the metadata.
        bind.InstanceDeduplicator.add_canonical(kinbody)
        kinbody.metadata = metadata

        return kinbody

    def ReadRobotDB(self, env, name):
        kinbody = self.ReadKinBodyDB(env, name)
        if not isinstance(kinbody, openravepy.Robot):
            kinbody.Destroy()
            raise IOError('Type "{0:s}" is not a robot.'.format(name))

        return kinbody
