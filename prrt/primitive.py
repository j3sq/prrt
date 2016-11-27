import numpy as np
import prrt.helper as helper
from typing import List, Tuple


class PointR2(object):
    """
    define a 2d point (x,y) in R2 euclidean space
    """

    def __init__(self, x=0., y=0.):
        self._position = np.array([x, y], dtype=float)

    def __str__(self):
        return '({0[0]:+.2f},{0[1]:+.2f})'.format(self._position)

    @property
    def x(self) -> float:
        return self._position[0]

    @x.setter
    def x(self, value: float):
        self._position[0] = value

    @property
    def y(self) -> float:
        return self._position[1]

    @y.setter
    def y(self, value: float):
        self._position[1] = value


class PoseR2S1(object):
    """
    define a pose (x,y,theta) in R2 S1 space
    """

    def __init__(self, x=0., y=0., theta=0.0):
        self.pose = np.array([x, y, theta], dtype=float)

    def __str__(self):
        return '({0:+.2f},{1:+.2f},{2:+.2f})'.format(self.x, self.y, np.rad2deg(self.theta))

    def __add__(self, other):
        result = PoseR2S1()
        result.x = self.x + other.x * np.cos(self.theta) - other.y * np.sin(self.theta)
        result.y = self.y + other.x * np.sin(self.theta) + other.y * np.cos(self.theta)
        result.theta = helper.wrap_to_npi_pi(self.theta + other.theta)
        return result

    def __sub__(self, other):
        result = PoseR2S1()
        result.x = (self.x - other.x) * np.cos(other.theta) + (self.y - other.y) * np.sin(other.theta)
        result.y = -(self.x - other.x) * np.sin(other.theta) + (self.y - other.y) * np.cos(other.theta)
        result.theta = helper.wrap_to_npi_pi(self.theta - other.theta)
        return result

    def __neg__(self):
        result = PoseR2S1()
        result.x = -self.x * np.cos(self.theta) - self.y * np.sin(self.theta)
        result.y = self.x * np.sin(self.theta) - self.y * np.cos(self.theta)
        result.theta = - self.theta
        return result

    def diff(self, other):
        result_pose = self.pose - other.pose
        result_pose_r2s1 = PoseR2S1()
        result_pose_r2s1.pose = result_pose
        result_pose_r2s1.theta = helper.wrap_to_npi_pi(result_pose_r2s1.theta)
        return result_pose_r2s1

    def copy_from(self, other):
        self.pose = np.copy(other.pose)

    def copy(self):
        return PoseR2S1(self.x, self.y, self.theta)

    def compose_point(self, p: PointR2) -> PointR2:
        gx = self.x + p.x * np.cos(self.theta) - p.y * np.sin(self.theta)
        gy = self.y + p.x * np.sin(self.theta) + p.y * np.cos(self.theta)
        return PointR2(gx, gy)

    def distance_2d(self, p) -> float:
        return np.sqrt((p.x - self.x) ** 2 + (p.y - self.y) ** 2)

    @property
    def x(self) -> float:
        return self.pose[0]

    @property
    def norm(self):
        return np.sqrt(self.pose[0] * self.pose[0] + self.pose[1] * self.pose[1])

    @x.setter
    def x(self, value: float):
        self.pose[0] = value

    @property
    def y(self) -> float:
        return self.pose[1]

    @y.setter
    def y(self, value: float):
        self.pose[1] = value

    @property
    def theta(self) -> float:
        return self.pose[2]

    @theta.setter
    def theta(self, value: float):
        self.pose[2] = helper.wrap_to_npi_pi(value)


class PoseR2S2(object):
    """
    define a pose (x,y,theta, phi) in R2 S2 space
    """

    def __init__(self, x=0., y=0., theta=0.0, phi=0.0):
        self._pose = np.array([x, y, theta, phi], dtype=float)

    def __str__(self):
        return '({0[0]:+.2f},{0[1]:+.2f},{0[2]:+.2f},{0[3]:+.2f})'.format(self._pose)


class CPoint(object):
    """
    Holds data about a trajectory point in configuration space
    """

    def __init__(self, pose: PoseR2S1 = None, t: float = 0., d: float = 0., v: float = 0., w: float = 0.):
        self._pose = pose
        self._t = t
        self._d = d
        self._v = v
        self._w = w

    @property
    def pose(self) -> PoseR2S1:
        return self._pose

    @pose.setter
    def pose(self, pose: PoseR2S1):
        self._pose = pose

    @property
    def t(self) -> float:
        return self._t

    @t.setter
    def t(self, t: float):
        self._t = t

    @property
    def d(self) -> float:
        return self._d

    @d.setter
    def d(self, d: float):
        self._d = d

    @property
    def v(self) -> float:
        return self._v

    @v.setter
    def v(self, v: float):
        self._v = v

    @property
    def w(self) -> float:
        return self._w

    @w.setter
    def w(self, w: float):
        self._w = w

    @property
    def x(self) -> float:
        return self._pose.x

    @x.setter
    def x(self, x: float):
        self._pose.x = x

    @property
    def y(self) -> float:
        return self._pose.y

    @y.setter
    def y(self, y: float):
        self._pose.y = y

    @property
    def theta(self) -> float:
        return self._pose.theta

    @theta.setter
    def theta(self, theta: float):
        self._pose.theta = theta


def get_bounding_box(points: List[PointR2]) -> Tuple[PointR2]:
    x = [point.x for point in points]
    y = [point.y for point in points]
    return PointR2(min(x), min(y)), PointR2(max(x), max(y))


def polygon_contains_point(polygon: List[PointR2], point: PointR2, bb: Tuple[PointR2] = None) -> bool:
    # Code ported to python from:
    # http://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon
    eps = 0.001
    segments = polygon_to_segments(polygon)
    if bb is None:
        bb = get_bounding_box(polygon)
    ray_start = PointR2(bb[0].x - eps, bb[0].y)
    ray = (ray_start, point)
    result = False
    for segment in segments:
        if ray_intersects_segment(segment, ray):
            result = not result
    return result


def polygon_to_segments(polygon: List[PointR2]) -> List[Tuple[PointR2]]:
    segments = []
    for k in range(len(polygon) - 1):
        segments.append((polygon[k], polygon[k + 1]))
    return segments


def ray_intersects_segment(segment: Tuple[PointR2], ray: Tuple[PointR2]) -> bool:
    # Convert vector 1 to a line (line 1) of infinite length.
    # We want the line in linear equation standard form: A*x + B*y + C = 0
    # See: http://en.wikipedia.org/wiki/Linear_equation
    a1 = segment[1].y - segment[0].y
    b1 = segment[0].x - segment[1].x
    c1 = (segment[1].x * segment[0].y) - (segment[0].x * segment[1].y)

    # Every point (x,y), that solves the equation above, is on the line,
    # every point that does not solve it, is not. The equation will have a
    # positive result if it is on one side of the line and a negative one
    # if is on the other side of it. We insert (x1,y1) and (x2,y2) of vector
    # 2 into the equation above.
    d1 = (a1 * ray[0].x) + (b1 * ray[0].y + c1)
    d2 = (a1 * ray[1].x) + (b1 * ray[1].y + c1)

    # If d1 and d2 both have the same sign, they are both on the same side
    # of our line 1 and in that case no intersection is possible. Careful,
    # 0 is a special case, that's why we don't test ">=" and "<=",
    # but "<" and ">".
    if d1 > 0 and d2 > 0:
        return False
    if d1 < 0 and d2 < 0:
        return False

    # The fact that vector 2 intersected the infinite line 1 above doesn't
    # mean it also intersects the vector 1. Vector 1 is only a subset of that
    # infinite line 1, so it may have intersected that line before the vector
    # started or after it ended. To know for sure, we have to repeat the
    # the same test the other way round. We start by calculating the
    # infinite line 2 in linear equation standard form.
    a2 = ray[1].y - ray[0].y
    b2 = ray[0].x - ray[1].x
    c2 = (ray[1].x * ray[0].y) - (ray[0].x * ray[1].y)

    # Calculate d1 and d2 again, this time using points of vector 1.
    d1 = (a2 * segment[0].x) + (b2 * segment[0].y + c2)
    d2 = (a2 * segment[1].x) + (b2 * segment[1].y + c2)
    # Again, if both have the same sign (and neither one is 0),
    # no intersection is possible.
    if d1 > 0 and d2 > 0:
        return False
    if d1 < 0 and d2 < 0:
        return False
    # If we get here, only two possibilities are left. Either the two
    # vectors intersect in exactly one point or they are collinear, which
    # means they intersect in any number of points from zero to infinite.
    # assert (a1 * b2) - (a2 * b1) != 0., 'Collinear segments not handled'
    if (a1 * b2) - (a2 * b1) == 0.:
        print('Collinear segments detected')
        return False
    # If they are not collinear, they must intersect in exactly one point.
    return True
