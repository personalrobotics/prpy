
class PlanningError(Exception):
    KNOWN_KWARGS = set(['deterministic'])

    def __init__(self, *args, **kwargs):
        super(PlanningError, self).__init__(*args)

        assert self.KNOWN_KWARGS.issuperset(kwargs.keys())
        self.deterministic = kwargs.get('deterministic', None)


class UnsupportedPlanningError(PlanningError):
    def __init__(self, *args):
        super(UnsupportedPlanningError, self).__init__(
            *args, deterministic=True)


class ConstraintViolationPlanningError(PlanningError):
    def __init__(self, 
                constraint_name, 
                threshold=None,
                violation_by=None, 
                base_message='Violates constraint',
                deterministic=None):
        self.constraint_name = constraint_name 
        self.threshold = threshold
        self.violation_by = violation_by

        super(ConstraintViolationPlanningError, self).__init__(
            '{:s}: {:s}'.format(
                base_message,
                self.constraint_name
            ),
            deterministic=deterministic
        )


class CollisionPlanningError(PlanningError):
    def __init__(self, link1, link2, base_message='Detected collision',
            deterministic=None):
        self.link1 = link1
        self.link2 = link2

        super(CollisionPlanningError, self).__init__(
            '{:s}: {:s} x {:s}.'.format(
                base_message,
                self._get_link_str(link1),
                self._get_link_str(link2)
            ),
            deterministic=deterministic
        )

    @classmethod
    def FromReport(cls, report, deterministic=None):
        return cls(report.plink1, report.plink2, deterministic=deterministic)

    @staticmethod
    def _get_link_str(link):
        if link is not None:
            return '<{:s}, {:s}>'.format(
                link.GetParent().GetName(), link.GetName())
        else:
            return '<unknown>'


class JointLimitError(PlanningError):
    def __init__(self, robot, dof_index, dof_value, dof_limit, description,
            deterministic=None):
        self.robot = robot
        self.dof_index = dof_index
        self.dof_value = dof_value
        self.dof_limit = dof_limit

        joint = robot.GetJointFromDOFIndex(dof_index)
        if dof_value < dof_limit:
            direction = 'lower'
            comparison = '<'
        else:
            direction = 'upper'
            comparison = '>'

        super(JointLimitError, self).__init__(
            'Robot "{robot_name:s}" joint "{joint_name:s} axis {joint_axis:d}'
            ' violates {direction:s} {description:s} limit:'
            ' {dof_value:.5f} {comparison:s} {dof_limit:.5f}'.format(
                robot_name=robot.GetName(),
                joint_name=joint.GetName(),
                joint_axis=dof_index - joint.GetDOFIndex(),
                dof_value=dof_value,
                dof_limit=dof_limit,
                comparison=comparison,
                direction=direction,
                description=description),
            deterministic=deterministic)

class SoftJointLimitError(JointLimitError):
    pass

class HardJointLimitError(JointLimitError):
    pass


class SelfCollisionPlanningError(CollisionPlanningError):
    pass


class TimeoutPlanningError(PlanningError):
    def __init__(self, timelimit=None, deterministic=None):
        if timelimit is not None:
            message = 'Exceeded {:.3f} s time limit.'.format(timelimit)
        else:
            message = 'Exceeded time limit.'

        super(TimeoutPlanningError, self).__init__(
            message, deterministic=deterministic)


class MetaPlanningError(PlanningError):
    """
    A metaplanning error indicates that a planning operation that calls one or
    more other planners internally failed due to the internal planning calls
    failing.
    """
    def __init__(self, message, errors, deterministic=None):
        PlanningError.__init__(self, message, deterministic=deterministic)
        self.errors = errors

    # TODO: Print the inner exceptions.


class ClonedPlanningError(PlanningError):
    """
    A cloned planning error indicates that planning failed because a
    ClonedPlanningMethod was unable to clone the environment successfully.

    This most commonly occurs when a planner attempts to clone while in
    collision, which can corrupt the environment before the planner would
    have a chance to detect the collision.
    """
    def __init__(self, cloning_error):
        super(ClonedPlanningError, self).__init__(
            "Failed to clone: {:s}".format(cloning_error),
            deterministic=True)
        self.error = cloning_error
