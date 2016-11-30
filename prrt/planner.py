from typing import List

import matplotlib.pyplot as plt
import numpy as np
from sortedcontainers import sorteddict

import prrt.helper as helper
from prrt.grid import WorldGrid
from prrt.primitive import PoseR2S1, PointR2
from prrt.ptg import PTG
from prrt.vehicle import Vehicle


class Node(object):
    """
    Represents a node in the RRT search tree
    """

    def __init__(self, pose: PoseR2S1, parent=None):
        self._pose = pose
        self.parent = parent
        self.edges_to_child = []  # type: List[Edge]

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, value: PoseR2S1):
        self._pose = value


class Edge(object):
    """
    An edge connecting two nodes
    """

    def __init__(self, ptg: PTG, k: float, d: float, parent: Node, end_pose: PointR2):
        self.ptg = ptg
        self.k = k
        self.d = d
        self.parent = parent
        self.end_pose = end_pose


class Tree(object):
    """
    Date structure to hold all nodes in RRT
    """

    def __init__(self, init_pose: PoseR2S1):
        root_node = Node(init_pose)
        self.nodes = [root_node]  # type: List[Node]
        self._edges = []  # type: List[Edge]

    def get_nearest_node(self, to_node: Node, ptg: PTG) -> (Node, float):
        d_min = float('inf')
        node_min = None
        for node in self.nodes:
            d = ptg.get_distance(node.pose, to_node.pose)
            if d < d_min:
                d_min = d
                node_min = node
        return node_min, d_min

    def insert_node_and_edge(self, parent: Node, child: Node, edge: Edge):
        self.nodes.append(child)
        self._edges.append(edge)
        parent.edges_to_child.append(edge)

    def plot_nodes(self, world: WorldGrid, goal: PointR2):
        fig, ax = plt.subplots()
        ax.matshow(world.omap, cmap=plt.cm.gray_r, origin='lower', interpolation='none')
        for node in self.nodes:
            x = world.x_to_ix(node.pose.x)
            y = world.y_to_iy(node.pose.y)
            ax.plot(x, y, 'bx')
        ax.plot(world.x_to_ix(goal.x), world.y_to_iy(goal.y), '+r')
        plt.savefig('./out/tree.png')
        plt.show()


class Planner(object):
    """
    Binds all pieces together and execute the main RRT algorithm
    """

    def __init__(self, ptgs: List[PTG]):
        self._ptgs = ptgs
        self.world = None  # type: WorldGrid
        self._init_pose = None  # type: PoseR2S1
        self._goal_pose = None  # type: PoseR2S1
        self.tree = None  # type: Tree

    def load_world_map(self, map_file, width: float, height: float):
        self.world = WorldGrid(map_file, width, height)
        self.world.build_obstacle_buffer()

    @staticmethod
    def transform_toTP_obstacles(ptg: PTG, obstacles_ws: List[PointR2], max_dist: float) -> List[float]:
        obs_TP = []  # type: List[float]
        for k in range(len(ptg.c_points)):
            obs_TP.append(ptg.distance_ref)
            # If you just turned 180deg, end there
            phi = ptg.c_points[k][-1].theta
            if abs(phi) >= np.pi * 0.95:
                obs_TP[k] = ptg.c_points[k][-1].d
        for obstacle in obstacles_ws:
            if abs(obstacle.x) > max_dist or abs(obstacle.y) > max_dist:
                continue
            collision_cell = ptg.obstacle_grid.cell_by_pos(obstacle)
            if collision_cell is None:
                continue
            # assert collision_cell is not None, 'collision cell is empty!'
            # get min_dist for the current k
            for kd_pair in collision_cell:
                if kd_pair.d < obs_TP[kd_pair.k]:
                    obs_TP[kd_pair.k] = kd_pair.d
        return obs_TP

    def solve(self, init_pose: PoseR2S1, goal_pose: PoseR2S1, goal_dist_tolerance=1.,
              goal_ang_tolerance=np.deg2rad(360)):
        assert self.world is not None, 'load_world_mmap must be called first'
        self._init_pose = init_pose
        self._goal_pose = goal_pose
        self.tree = Tree(init_pose)
        solution_found = False
        counter = 0
        min_goal_dist_yet = float('inf')
        while not solution_found and counter < 2000:
            counter += 1
            rand_pose = self.world.get_random_pose(goal_pose)
            candidate_new_nodes = sorteddict.SortedDict()
            rand_node = Node(rand_pose)
            for ptg in self._ptgs:
                ptg_nearest_node, ptg_d_min = self.tree.get_nearest_node(rand_node, ptg)
                if ptg_nearest_node is None:
                    print('PTG {0} can\'t find nearest pose to {1}'.format(ptg, rand_node))
                    continue
                ptg_nearest_pose = ptg_nearest_node.pose
                rand_pose_rel = rand_pose - ptg_nearest_pose
                D_max = min(2., ptg.distance_ref)  # ToDo : make 2. a configurable parameter
                is_exact, k_rand, d_rand = ptg.inverse_WS2TP(rand_pose_rel)
                d_rand *= ptg.distance_ref
                max_dist_for_obstacles = 1.5 * ptg.distance_ref
                obstacles_rel = self.world.transform_point_cloud(ptg_nearest_pose, max_dist_for_obstacles)
                obstacles_TP = self.transform_toTP_obstacles(ptg, obstacles_rel, max_dist_for_obstacles)
                d_free = obstacles_TP[k_rand]
                d_new = min(D_max, d_free)
                if d_free >= d_new:
                    # get cpoint at d_new
                    cpoint = ptg.get_cpoint_at_d(d_new, k_rand)
                    new_pose_rel = cpoint.pose.copy()
                    new_pose = ptg_nearest_pose + new_pose_rel  # type: PoseR2S1
                    accept_this_node = True
                    goal_dist = new_pose.distance_2d(goal_pose)
                    goal_ang = abs(helper.angle_distance(new_pose.theta, goal_pose.theta))
                    is_acceptable_goal = goal_dist < goal_dist_tolerance and goal_ang < goal_ang_tolerance
                    new_nearest_node = None  # type: Node
                    if not is_acceptable_goal:
                        new_node = Node(new_pose)
                        new_nearest_node, new_nearest_dist = self.tree.get_nearest_node(new_node, ptg)
                        if new_nearest_node is not None:
                            new_nearest_ang = abs(helper.angle_distance(new_pose.theta, new_nearest_node.pose.theta))
                            accept_this_node = new_nearest_dist >= 0.1 or new_nearest_ang >= 0.35
                            # ToDo: make 0.1 and 0.35 configurable parameters
                    if not accept_this_node:
                        continue
                    new_edge = Edge(ptg, k_rand, d_new, ptg_nearest_node, new_pose)
                    candidate_new_nodes.update({d_new: new_edge})
                    print('Candidate node found')
                else:  # path is not free
                    print('Obstacle ahead!')
                    # do nothing for now
                    pass
            if len(candidate_new_nodes) > 0:
                best_edge = candidate_new_nodes.peekitem(-1)[1]  # type : Edge
                new_state_node = Node(best_edge.end_pose, best_edge.parent)
                self.tree.insert_node_and_edge(best_edge.parent, new_state_node, best_edge)
                print('new node added to tree from ptg {0}'.format(best_edge.ptg.name))
                print(new_state_node.pose, ptg_d_min)

                goal_dist = best_edge.end_pose.distance_2d(goal_pose)
                goal_ang = abs(helper.angle_distance(best_edge.end_pose.theta, goal_pose.theta))
                is_acceptable_goal = goal_dist < goal_dist_tolerance and goal_ang < goal_ang_tolerance
                min_goal_dist_yet = min(goal_dist, min_goal_dist_yet)
                if is_acceptable_goal:
                    print('goal reached!')
                    return
                    # To do: continue PlannerRRT_SE2_TPS.cpp at line 415
                print(counter)
        print('Done!')
        print('Minimum distance to goal reached is {0}'.format(min_goal_dist_yet))

    def trace_solution(self, vehicle: Vehicle, goal: PoseR2S1 = None):
        child_node = self.tree.nodes[-1]
        trajectory = []  # type #: List[Edge]
        while True:
            parent_node = child_node.parent
            if parent_node is None:
                break
            trajectory.append(self.get_trajectory_edge(parent_node, child_node))
            child_node = parent_node
        fig, ax = plt.subplots()
        frame = 0
        for i in range(len(trajectory) - 1, -1, -1):
            # plot the vehicle
            edge = trajectory[i]
            start_pose = edge.parent.pose.copy()
            color = 'b'
            vertex_count = len(vehicle.shape)
            for d in np.arange(0., edge.d, 0.2):
                ax.matshow(self.world.omap, cmap=plt.cm.gray_r, origin='lower')
                c_point = edge.ptg.get_cpoint_at_d(d, edge.k)
                current_pose = start_pose + c_point.pose
                vertices = vehicle.get_vertices_at_pose(current_pose)
                for j in range(vertex_count):
                    a = vertices[j % vertex_count]
                    b = vertices[(j + 1) % vertex_count]
                    ia = PointR2(self.world.x_to_ix(a.x), self.world.y_to_iy(a.y))
                    ib = PointR2(self.world.x_to_ix(b.x), self.world.y_to_iy(b.y))
                    ax.plot([ia.x, ib.x], [ia.y, ib.y], color)
                if goal is not None:
                    ax.plot(self.world.x_to_ix(goal.x), self.world.y_to_iy(goal.y), '+r')
                print('Saving frame {0}'.format(frame))
                plt.savefig('./out/frame{0:04d}.png'.format(frame))
                plt.cla()
                frame += 1

    @staticmethod
    def get_trajectory_edge(parent: Node, child: Node) -> Edge:
        for edge in parent.edges_to_child:
            if edge.end_pose == child.pose:
                return edge
