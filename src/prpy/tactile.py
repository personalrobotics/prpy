import numpy, openravepy

def toH(x):
    return numpy.concatenate((x, [ 1.0 ]))

def fromH(x):
    return x[0:3]

class TactileCell(object):
    def __init__(self, name, origin, normal):
        if not (origin.shape == (3,)):
            raise ValueError('Origin must be a three-element vector.')
        elif not (normal.shape == (3,)):
            raise ValueError('Normal must be a three-element vector.')

        self.name = name
        self.origin = numpy.array(origin, dtype='float')
        self.normal = numpy.array(normal, dtype='float')

    def __repr__(self):
        return 'TactileCell(name={0:s}, origin={1:s}, normal={2:s})'.format(
                   self.name, self.origin, self.normal)

class TactileArray(object):
    def __init__(self, link_name, offset):
        self.cells = list()
        self.link_name = link_name
        self.offset = numpy.array(offset, dtype='float')

class TactileSensor(object):
    def __init__(self):
        self.pads = list()

    def from_yaml(self, path):
        self.pads = list()

        with open(path, 'rb') as stream:
            import yaml
            tactile_yaml = yaml.load(stream)

        for parent_link, pad_yaml in tactile_yaml.items():
            # Extract the link offset. This, combined with the link frame,
            # specifies the coordinate frame of the cell origins and normals.
            offset_quaternion =  numpy.array(pad_yaml['offset']['orientation'], dtype='float')
            offset_pose = openravepy.matrixFromQuat(offset_quaternion)
            offset_pose[0:3, 3] = numpy.array(pad_yaml['offset']['position'], dtype='float')

            tactile_array = TactileArray(parent_link, offset_pose)
            cells_yaml = zip(pad_yaml['origin'], pad_yaml['normal'])

            for index, (origin_yaml, normal_yaml) in enumerate(cells_yaml):
                cell_name = '{0:s}_cell{1:d}'.format(parent_link, index)
                origin = numpy.array(origin_yaml, dtype='float')
                normal = numpy.array(normal_yaml, dtype='float')

                print normal_yaml

                cell = TactileCell(cell_name, origin, normal)
                tactile_array.cells.append(cell)

            self.pads.append(tactile_array)

    def render(self, robot, origins=True, points=True, normals=True, length=0.01):
        handles = list()
        lines = list()

        for tactile_array in self.pads:
            link = robot.GetLink(tactile_array.link_name)
            if link is None:
                raise openravepy.openrave_exception('There is no link named {0:s}.'.format(tactile_array.link_name))

            # Compute the origin of the tactile array in the world frame.
            link_pose = link.GetTransform()
            offset_pose = numpy.dot(link_pose, tactile_array.offset)

            # Render the origin of the tactile array.
            if origins:
                handle = openravepy.misc.DrawAxes(robot.GetEnv(), offset_pose, dist=0.02, linewidth=2)
                handles.append(handle)

            # Compute a normal vector for each cell.
            if normals:
                for cell in tactile_array.cells:
                    cell_origin = fromH(numpy.dot(offset_pose, toH(cell.origin)))
                    cell_normal = numpy.dot(offset_pose[0:3, 0:3], cell.normal)
                    lines += [ cell_origin, cell_origin + length * cell_normal ]

        # Render the normal vectors.
        if normals:
            lines = numpy.array(lines, dtype='float')
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
