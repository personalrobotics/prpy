import numpy, openravepy

class TactileArray(object):
    def __init__(self, offset, origins, normals):
        self.offset = numpy.array(offset, dtype='float')
        self.origins = numpy.array(origins, dtype='float')
        self.normals = numpy.array(normals, dtype='float')

    def __repr__(self):
        return 'TactileArray(offset={0:r}, origins=<{1:d}x{2:d} array>, normals=<{3:d}x{4:d} array>)'.format(
            self.offset, self.origins.shape[0], self.origins.shape[1],
                         self.normals.shape[0], self.origins.shape[1])

    def __len__(self):
        return self.origins.shape[0]

    def get_geometry(self, link_pose):
        offset = self.get_offset(link_pose)
        origins = numpy.dot(offset[0:3, 0:3], self.origins.T) + offset[0:3, 3].reshape((3, 1))
        normals = numpy.dot(offset[0:3, 0:3], self.normals.T)
        return origins.T, normals.T

    def get_offset(self, link_pose):
        return numpy.dot(link_pose, self.offset)

    @classmethod
    def from_yaml(cls, array_yaml):
        offset_quaternion =  numpy.array(array_yaml['offset']['orientation'], dtype='float')
        offset_pose = openravepy.matrixFromQuat(offset_quaternion)
        offset_pose[0:3, 3] = numpy.array(array_yaml['offset']['position'], dtype='float')
        return cls(offset_pose, array_yaml['origin'], array_yaml['normal'])

class TactileSensor(object):
    def __init__(self):
        self.arrays = dict()

    def get_geometry(self, robot):
        all_origins = list()
        all_normals = list()

        for link_name, tactile_array in self.arrays.items():
            link_pose = robot.GetLink(link_name).GetTransform()
            array_origins, array_normals = tactile_array.get_geometry(link_pose)
            all_origins.append(array_origins)
            all_normals.append(array_normals)

        return numpy.vstack(all_origins), numpy.vstack(all_normals)

    def render_values(self, robot, values, scale=1.0, color=None, linewidth=2):
        all_lines = list()

        if color is None:
            color = numpy.array([ 1., 0., 0., 1. ])

        if isinstance(values, dict):
            all_values = list()
            for link_name in self.arrays.keys():
                all_values.extend(values[link_name])
        else:
            all_values = values

        num_cells = len(all_values)
        all_values = numpy.array(all_values, dtype='float')
        origins, normals = self.get_geometry(robot)
        lines = numpy.empty((2 * num_cells, 3))
        lines[0::2, :] = origins
        lines[1::2, :] = origins + scale * all_values.reshape((num_cells, 1)) * normals
        return robot.GetEnv().drawlinelist(lines, linewidth, color)

    def render_cells(self, robot, origins=True, normals=True, color=None, linewidth=2, length=0.01):
        if color is None:
            color = numpy.array([ 1., 1., 0., 1. ])

        all_origins, all_normals = self.get_geometry(robot)
        handles = list()

        if normals:
            lines = numpy.empty((2 * all_origins.shape[0], 3))
            lines[0::2, :] = all_origins
            lines[1::2, :] = all_origins + length * all_normals
            handle = robot.GetEnv().drawlinelist(lines, linewidth, color)
            handles.append(handle)

        if origins:
            for tactile_array in self.arrays.values():
                handle = openravepy.misc.DrawAxes(robot.GetEnv(), tactile_array.offset,
                                                  dist=0.02, linewidth=linewidth)
                handles.append(handle)

        return handles

    @classmethod
    def from_yaml(cls, path):
        # Load the tactile cell origins and normals from YAML.
        with open(path, 'rb') as stream:
            import yaml
            tactile_yaml = yaml.load(stream)

        # Create a TactileArray object for each entry in the file.
        sensor = cls()
        for link_name, array_yaml in tactile_yaml.items():
            sensor.arrays[link_name] = TactileArray.from_yaml(array_yaml)

        return sensor
