
class PlanningError(Exception):
    pass


class UnsupportedPlanningError(PlanningError):
    pass


class ConstraintViolationPlanningError(PlanningError):
    pass


class CollisionPlanningError(PlanningError):
    def __init__(self, link1, link2, base_message='Detected collision'):
        self.link1 = link1
        self.link2 = link2

        super(CollisionPlanningError, self).__init__(
            '{:s}: {:s} x {:s}.'.format(
                base_message,
                self._get_link_str(link1),
                self._get_link_str(link2)
            )
        )

    @classmethod
    def FromReport(cls, report):
        return cls(report.plink1, report.plink2)

    @staticmethod
    def _get_link_str(link):
        if link is not None:
            return '<{:s}, {:s}>'.format(
                link.GetParent().GetName(), link.GetName())
        else:
            return '<unknown>'


class SelfCollisionPlanningError(CollisionPlanningError):
    pass


class TimeoutPlanningError(PlanningError):
    def __init__(self, timelimit=None):
        if timelimit is not None:
            message = 'Exceeded {:.3f} s time limit.'.format(timelimit)
        else:
            message = 'Exceeded time limit.'

        super(TimeoutPlanningError, self).__init__(message)
