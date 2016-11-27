from abc import ABCMeta, abstractmethod
from prrt.primitive import PointR2, PoseR2S1
from typing import Tuple, List
import numpy as np


class Vehicle(metaclass=ABCMeta):
    """
    Manage a vehicle with a specific car_vertices(polygon) and kinematic constraints
    """

    def __init__(self, v_max: float, alpha_max: float, a_max: float, w_max: float):
        """

        :type v_max: float maximum absolute linear velocity
        :type alpha_max: float maximum absolute steering angle
        :type a_max: float maximum absolute linear velocity
        :type w_max: float maximum absolute rotational velocity
        """
        self.v_max = v_max
        self.alpha_max = alpha_max
        self.a_max = a_max
        self.w_max = w_max
        self.shape = ()  # type: Tuple[PointR2]
        self.alpha_resolution = np.deg2rad(5)

    @abstractmethod
    def set_vertices(self, polygon: Tuple[PointR2]):
        """
        Sets the vertices of vehicle. Each vertex is a 2D point (x,y)
        vertices are relative to the robot center
        """
        self.shape = polygon

    def get_vertex(self, idx: int) -> PointR2:
        return self.shape[idx]

    def get_vertices_at_pose(self, pose: PoseR2S1) -> List[PointR2]:
        result = []
        for vertex in self.shape:
            point = PointR2()
            point.x = pose.x + np.cos(pose.theta) * vertex.x - np.sin(pose.theta) * vertex.y
            point.y = pose.y + np.sin(pose.theta) * vertex.x + np.cos(pose.theta) * vertex.y
            result.append(point)
        return result


class Car(Vehicle):
    """
    A non-articulated vehicle with a rectangle shape defined by four vertices
    """
    def set_vertices(self, vertices: Tuple[PointR2]):
        assert len(vertices) == 4, '4 vertices expected, received {0}'.format(len(vertices))
        super(Car, self).set_vertices(vertices)
