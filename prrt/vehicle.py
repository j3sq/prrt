from prrt.primitive import PointR2, PoseR2S2
from typing import List
from math import cos, sin, pi as PI, radians as rad
from prrt.grid import WorldGrid


class ArticulatedVehicle(object):
    """
    Articulated vehicle represented by tractor, link and trailer
    Vehicle origin is the center of the tractor, see blow fig
    """

    def __init__(self, config: dict):
        # check the provided config file for details on the variables initialized
        self.v_max = config['v_max']  # type: float
        self.alpha_max = rad(config['alpha_max'])  # type: float
        self.w_max = rad(config['w_max'])  # type: float
        self.phi_max = rad(config['phi_max'])  # type: float
        self.tractor_w = config['tractor_w']  # type: float
        self.tractor_l = config['tractor_l']  # type: float
        self.link_l = config['link_l']  # type: float
        self.trailer_w = config['trailer_w']  # type: float
        self.trailer_l = config['trailer_l']  # type: float
        self.__phi = 0.  # keeps track of changes to __phi in order to update vehicle shape
        # vertices numbered as shown below
        #
        #       8----------------7       2-------1
        #       |                |       |       |
        # Trl_W |                5-------4   o   | Trk_W
        #       |                |  L_L  |       |
        #       9----------------6       3-------0
        #              Trl_L               Trk_L
        o = PoseR2S2(0., 0., 0., 0.)
        p0 = o.compose_point(PointR2(self.tractor_l / 2., -self.tractor_w / 2.))
        p1 = o.compose_point(PointR2(self.tractor_l / 2., self.tractor_w / 2.))
        p2 = o.compose_point(PointR2(-self.tractor_l / 2., self.tractor_w / 2.))
        p3 = o.compose_point(PointR2(-self.tractor_l / 2., -self.tractor_w / 2.))
        p4 = o.compose_point(PointR2(-self.tractor_l / 2., 0.))
        p5 = o.compose_point(PointR2(-self.tractor_l / 2. - self.link_l, 0.))
        hitch_pose = PoseR2S2(p5.x, p5.y, self.__phi)
        p6 = hitch_pose.compose_point(PointR2(0, -self.trailer_w / 2.))
        p7 = hitch_pose.compose_point(PointR2(0, self.trailer_w / 2.))
        p8 = hitch_pose.compose_point(PointR2(-self.trailer_l, self.trailer_w / 2.))
        p9 = hitch_pose.compose_point(PointR2(-self.trailer_l, -self.trailer_w / 2.))
        self.shape = (p0, p1, p2, p3, p4, p5, p6, p7, p8, p9)
        self.__hitch_pose = hitch_pose

    def update_shape(self, phi: float):
        # adjust the trailer vertices based on articulation angel
        self.__hitch_pose.theta = -phi
        p6 = self.__hitch_pose.compose_point(PointR2(0, -self.trailer_w / 2.))
        p7 = self.__hitch_pose.compose_point(PointR2(0, self.trailer_w / 2.))
        p8 = self.__hitch_pose.compose_point(PointR2(-self.trailer_l, self.trailer_w / 2.))
        p9 = self.__hitch_pose.compose_point(PointR2(-self.trailer_l, -self.trailer_w / 2.))
        self.shape = (*self.shape[:6], p6, p7, p8, p9)

    def get_vertex(self, idx: int) -> PointR2:
        return self.shape[idx]

    def get_vertices_at_pose(self, pose: PoseR2S2) -> List[PointR2]:
        if pose.phi != self.__phi:
            self.update_shape(pose.phi)
        result = []
        for vertex in self.shape:
            point = PointR2()
            point.x = pose.x + cos(pose.theta) * vertex.x - sin(pose.theta) * vertex.y
            point.y = pose.y + sin(pose.theta) * vertex.x + cos(pose.theta) * vertex.y
            result.append(point)
        return result

    def get_tractor_vertices_at_pose(self, pose: PoseR2S2) -> List[PointR2]:
        result = []
        for i in range(4):
            vertex = self.shape[i]
            point = PointR2()
            point.x = pose.x + cos(pose.theta) * vertex.x - sin(pose.theta) * vertex.y
            point.y = pose.y + sin(pose.theta) * vertex.x + cos(pose.theta) * vertex.y
            result.append(point)
        return result

    def get_trailer_vertices_at_pose(self, pose: PoseR2S2) -> List[PointR2]:
        return self.get_vertices_at_pose(pose)[6:10]

    def execute_motion(self, pose: PoseR2S2, K: int, w: float, dt: float) -> PoseR2S2:
        if K == 1:
            new_pose = self._sim_move_forward(pose, w, dt)
        elif K == -1:
            # rev_pose = self._sim_reverse(pose, w, dt)
            new_pose = self._sim_move_reverse(pose, w, dt)
            # new_pose = self._sim_reverse(new_rev_pose)
        return new_pose

    def _sim_move_reverse(self, pose: PoseR2S2, w: float, dt: float):
        # Transform to simulated car pulling the trailer backwards
        v = self.v_max
        x_rev = pose.x - (self.trailer_l / 2.) * cos(pose.theta) - \
                (self.tractor_l / 2 + self.link_l) * cos(pose.theta + pose.phi)
        y_rev = pose.y - (self.trailer_l / 2) * sin(pose.theta) - \
                (self.tractor_l / 2 + self.link_l) * sin(pose.theta + pose.phi)
        theta_rev = pose.theta + pose.phi + PI
        phi_rev = -pose.phi

        # Move forward one step
        x_rev += v * dt * cos(theta_rev)
        y_rev += v * dt * sin(theta_rev)
        theta_rev += dt * w
        phi_rev += dt * ((v / (self.trailer_l / 2)) * sin(phi_rev) - (
            (self.tractor_l / 2 + self.link_l) * w / (self.trailer_l / 2)) * cos(phi_rev) - w)

        # Transform back
        new_pose = PoseR2S2()
        new_pose.phi = - phi_rev
        new_pose.theta = theta_rev - new_pose.phi - PI
        new_pose.x = x_rev + (self.trailer_l / 2.) * cos(new_pose.theta) + \
                     (self.tractor_l / 2 + self.link_l) * cos(new_pose.theta + new_pose.phi)
        new_pose.y = y_rev + (self.trailer_l / 2) * sin(new_pose.theta) + \
                     (self.tractor_l / 2 + self.link_l) * sin(new_pose.theta + new_pose.phi)

        return new_pose

    def _sim_move_forward(self, pose: PoseR2S2, w: float, dt: float) -> (PoseR2S2, float):
        final_pose = PoseR2S2()
        v = self.v_max
        final_pose.x = pose.x + v * dt * cos(pose.theta)
        final_pose.y = pose.y + v * dt * sin(pose.theta)
        final_pose.theta = pose.theta + w * dt
        final_pose.phi = pose.phi - dt * ((v / (self.trailer_l / 2)) * sin(pose.phi) - (
            (self.tractor_l / 2 + self.link_l) * w / (self.trailer_l / 2)) * cos(pose.phi) - w)
        return final_pose

    def plot(self, axes, pose, world: WorldGrid, color='b'):
        vertices = self.get_vertices_at_pose(pose)
        for j in range(len(vertices) - 1):
            a = vertices[j]
            b = vertices[j + 1]
            if world is None:
                axes.plot([a.x, b.x], [a.y, b.y], color)
            else:
                ia = PointR2(world.x_to_ix(a.x), world.y_to_iy(a.y))
                ib = PointR2(world.x_to_ix(b.x), world.y_to_iy(b.y))
                axes.plot([ia.x, ib.x], [ia.y, ib.y], color)

        a = vertices[3]
        b = vertices[0]
        if world is None:
            axes.plot([a.x, b.x], [a.y, b.y], color)
        else:
            ia = PointR2(world.x_to_ix(a.x), world.y_to_iy(a.y))
            ib = PointR2(world.x_to_ix(b.x), world.y_to_iy(b.y))
            axes.plot([ia.x, ib.x], [ia.y, ib.y], color)

        a = vertices[9]
        b = vertices[6]
        if world is None:
            axes.plot([a.x, b.x], [a.y, b.y], color)
        else:
            ia = PointR2(world.x_to_ix(a.x), world.y_to_iy(a.y))
            ib = PointR2(world.x_to_ix(b.x), world.y_to_iy(b.y))
            axes.plot([ia.x, ib.x], [ia.y, ib.y], color)
