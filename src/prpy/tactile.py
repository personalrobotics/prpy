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

    def get_origins(self, link_pose):
        offset = self.get_offset(link_pose)
        origins = numpy.dot(offset[0:3, 0:3], self.origins.T) + offset[0:3, 3].reshape((3, 1))
        return origins.T

    def get_normals(self, link_pose):
        offset = self.get_offset(link_pose)
        normals = numpy.dot(offset[0:3, 0:3], self.normals.T)
        return normals.T

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

    def render_cells(self, robot, origins=True, normals=True, length=0.01):
        handles = list()
        all_lines = list()

        for link_name, tactile_array in self.arrays.items():
            # Tactile cells are relative to the link.
            link = robot.GetLink(link_name)
            link_pose = link.GetTransform()

            # Render the origin of the tactile array.
            if origins:
                handle = openravepy.misc.DrawAxes(robot.GetEnv(), tactile_array.offset,
                                                  dist=0.02, linewidth=2)
                handles.append(handle)

            # Render a normal vector for each cell.
            if normals:
                num_cells = len(tactile_array)
                cell_origins = tactile_array.get_origins(link_pose)
                cell_normals = tactile_array.get_normals(link_pose)

                array_lines = numpy.empty((2 * num_cells, 3))
                array_lines[0::2, :] = cell_origins
                array_lines[1::2, :] = cell_origins + length * cell_normals
                all_lines.append(array_lines)

        # Render the normal vectors.
        if normals:
            lines = numpy.vstack(all_lines)
            color = numpy.array([ 1., 1., 0., 1. ])
            handle = robot.GetEnv().drawlinelist(lines, 2, color)
            handles.append(handle)

        return handles

if __name__ == '__main__':

    env = openravepy.Environment()
    hand = env.ReadRobotURI('robots/iHY/irobot_reduced.kinbody.xml')
    env.Add(hand)
    env.SetViewer('qtcoin')

    sensor = TactileSensor()
    sensor.from_yaml('config/ihy_tactile.yaml')
    handles = sensor.render(hand)
