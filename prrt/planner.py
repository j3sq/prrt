from typing import List

import matplotlib.pyplot as plt
import numpy as np
from sortedcontainers import sorteddict

import prrt.helper as helper
from prrt.grid import WorldGrid
from prrt.primitive import PoseR2S2, PointR2
from prrt.ptg import PTG, APTG
from math import degrees as deg
from prrt.vehicle import ArticulatedVehicle


class Node(object):
    """
    Represents a node in the RRT search tree
    """

    def __init__(self, ptg: PTG, pose: PoseR2S2, parent=None):
        self.pose = pose
        self.parent = parent
        self.ptg = ptg
        self.edges_to_child = []  # type: List[Edge]

    def __str__(self):
        return str(self.pose)


class Edge(object):
    """
    An edge connecting two nodes
    """

    def __init__(self, ptg: PTG, k: float, d: float, parent: Node, end_pose: PoseR2S2):
        self.ptg = ptg
        self.k = k
        self.d = d
        self.parent = parent
        self.end_pose = end_pose


class Tree(object):
    """
    Date structure to hold all nodes in RRT
    """

    def __init__(self, init_pose: PoseR2S2):
        root_node = Node(ptg=None, pose=init_pose)
        self.nodes = [root_node]  # type: List[Node]
        self._edges = []  # type: List[Edge]

    def get_ptg_nearest_node(self, to_node: Node, ptg: PTG) -> (PTG, Node, float):
        d_min = float('inf')
        node_min = None
        for node in self.nodes:
            if node.ptg is ptg:  # only search nodes reachable by the current ptg
                # Only do the the expensive ptg.get_distance when needed
                if abs(node.pose.x - to_node.pose.x) > d_min:
                    continue
                if abs(node.pose.y - to_node.pose.y) > d_min:
                    continue
                d = ptg.get_distance(node.pose, to_node.pose)
                if d < d_min:
                    d_min = d
                    node_min = node

        return node_min, d_min

    def get_aptg_nearest_node(self, to_node: Node, aptg: APTG) -> (PTG, Node, float):
        d_min = float('inf')
        node_min = None
        node_ptg = None
        for node in self.nodes:
            # Only do the the expensive ptg.get_distance when needed
            if abs(node.pose.x - to_node.pose.x) > d_min:
                continue
            if abs(node.pose.y - to_node.pose.y) > d_min:
                continue
            ptg = aptg.ptg_at_phi(node.pose.phi)
            d = ptg.get_distance(node.pose, to_node.pose)
            if d < d_min:
                d_min = d
                node_min = node
                node_ptg = ptg

        return node_ptg, node_min, d_min

    def insert_node_and_edge(self, parent: Node, child: Node, edge: Edge):
        self.nodes.append(child)
        self._edges.append(edge)
        parent.edges_to_child.append(edge)

    def plot_nodes(self, world: WorldGrid, goal: PointR2 = None, file_name=None):
        fig, ax = plt.subplots()
        ax.matshow(world.omap, cmap=plt.cm.gray_r, origin='lower', interpolation='none')
        for node in self.nodes:
            x = world.x_to_ix(node.pose.x)
            y = world.y_to_iy(node.pose.y)
            ax.plot(x, y, 'bx')
        if goal is not None:
            ax.plot(world.x_to_ix(goal.x), world.y_to_iy(goal.y), '+r')
        if file_name is None:
            plt.savefig('./out/tree.png')
        else:
            plt.savefig(file_name)
            # plt.show()


class Planner(object):
    """
    Binds all pieces together and execute the main RRT algorithm
    """

    def __init__(self, config: dict):
        self.aptgs = []  # Type:List[APTG]
        self.world = None  # type: WorldGrid
        self.init_pose = None  # type: PoseR2S2
        self.goal_pose = None  # type: PoseR2S2
        self.tree = None  # type: Tree
        self.config = config

    def load_world_map(self, map_file, width: float, height: float):
        self.world = WorldGrid(map_file, width, height)
        self.world.build_obstacle_buffer()

    def load_aptgs(self, files: List[str]):
        for file in files:
            self.aptgs.append(helper.load_object(file))

    def setup(self):
        aptgs_files = self.config['aptg_files']
        self.load_aptgs(aptgs_files)
        map_file = self.config['world_map_file']
        width = self.config['world_width']
        height = self.config['world_height']
        self.load_world_map(map_file, width, height)

    @staticmethod
    def transform_toTP_obstacles(ptg: PTG, obstacles_ws: List[PointR2], max_dist: float) -> List[float]:
        # obs_TP = []  # type: List[float]
        # for k in range(len(ptg.cpoints)):
        #     obs_TP.append(ptg.distance_ref)
            # If you just  turned 180deg, end there
            # if len(ptg.cpoints[k]) == 0:
            #     obs_TP[k] = 0.  # Invalid configuration
            #     continue
            # phi = ptg.cpoints[k][-1].theta
            # if abs(phi) >= np.pi * 0.95:
            #     obs_TP[k] = ptg.cpoints[k][-1].d
        obs_TP = [ptg.distance_ref]*len(ptg.cpoints)  # type: List[float]
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

    def solve(self):
        self.setup()  # load aptgs and world map
        init_pose = PoseR2S2.from_dict(self.config['init_pose'])
        goal_pose = PoseR2S2.from_dict(self.config['goal_pose'])
        self.tree = Tree(init_pose)
        goal_dist_tolerance = self.config['goal_dist_tolerance']
        goal_ang_tolerance = self.config['goal_ang_tolerance']
        debug_tree_state = self.config['debug_tree_state']
        debug_tree_state_file = self.config['debug_tree_state_file']
        obs_R = self.config['obs_R']
        D_max = self.config['D_max']
        solution_found = False
        max_count = self.config['max_count']
        counter = 0
        min_goal_dist_yet = float('inf')
        while not solution_found and len(self.tree.nodes) < max_count:
            counter += 1
            rand_pose = self.world.get_random_pose(goal_pose)
            candidate_new_nodes = sorteddict.SortedDict()
            rand_node = Node(ptg=None, pose=rand_pose)
            for aptg in self.aptgs:
                ptg, ptg_nearest_node, ptg_d_min = self.tree.get_aptg_nearest_node(rand_node, aptg)
                if ptg_nearest_node is None:
                    print('APTG {0} can\'t find nearest pose to {1}'.format(aptg.name, rand_node))
                    continue
                ptg_nearest_pose = ptg_nearest_node.pose
                rand_pose_rel = rand_pose - ptg_nearest_pose
                d_max = min(D_max, ptg.distance_ref)
                is_exact, k_rand, d_rand = ptg.inverse_WS2TP(rand_pose_rel)
                d_rand *= ptg.distance_ref
                max_dist_for_obstacles = obs_R * ptg.distance_ref
                obstacles_rel = self.world.transform_point_cloud(ptg_nearest_pose, max_dist_for_obstacles)
                obstacles_TP = self.transform_toTP_obstacles(ptg, obstacles_rel, max_dist_for_obstacles)
                d_free = obstacles_TP[k_rand]
                d_new = min(d_max, d_rand)
                if debug_tree_state > 0 and counter % debug_tree_state == 0:
                    self.tree.plot_nodes(self.world, ptg_nearest_pose,
                                         '{0}{1:04d}.png'.format(debug_tree_state_file, counter))
                # Skip if the current ptg and alpha (k_ran) can't reach this point
                if ptg.cpoints[k_rand][-1].d < d_new:
                    print('Node leads to invalid trajectory. Node Skipped!')
                    continue

                if d_free >= d_new:
                    # get cpoint at d_new
                    cpoint = ptg.get_cpoint_at_d(d_new, k_rand)
                    new_pose_rel = cpoint.pose.copy()
                    new_pose = ptg_nearest_pose + new_pose_rel  # type: PoseR2S2
                    accept_this_node = True
                    goal_dist = new_pose.distance_2d(goal_pose)
                    goal_ang = abs(helper.angle_distance(new_pose.theta, goal_pose.theta))
                    is_acceptable_goal = goal_dist < goal_dist_tolerance and goal_ang < goal_ang_tolerance
                    new_nearest_node = None  # type: Node
                    if not is_acceptable_goal:
                        new_node = Node(ptg, new_pose)
                        new_nearest_ptg, new_nearest_node, new_nearest_dist = self.tree.get_aptg_nearest_node(new_node,
                                                                                                              aptg)
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
                new_state_node = Node(best_edge.ptg, best_edge.end_pose, best_edge.parent)
                self.tree.insert_node_and_edge(best_edge.parent, new_state_node, best_edge)
                print('new node added to tree from ptg {0}'.format(best_edge.ptg.name))
                goal_dist = best_edge.end_pose.distance_2d(goal_pose)
                print(new_state_node.pose, goal_dist)
                goal_ang = abs(helper.angle_distance(best_edge.end_pose.theta, goal_pose.theta))
                is_acceptable_goal = goal_dist < goal_dist_tolerance and goal_ang < goal_ang_tolerance
                min_goal_dist_yet = min(goal_dist, min_goal_dist_yet)
                if is_acceptable_goal:
                    print('goal reached!')
                    break
                    # To do: continue PlannerRRT_SE2_TPS.cpp at line 415
                print(counter)
        print('Done!')
        print('Minimum distance to goal reached is {0}'.format(min_goal_dist_yet))
        if self.config['plot_tree_file'] != '':
            self.tree.plot_nodes(self.world, goal_pose, self.config['plot_tree_file'])
        if self.config['plot_solution'] != '':
            self.trace_solution(self.aptgs[0].vehicle, goal_pose, self.config['plot_solution'])

    def trace_solution(self, vehicle: ArticulatedVehicle, goal: PoseR2S2 = None, file_name='frame'):
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
        ax.matshow(self.world.omap, cmap=plt.cm.gray_r, origin='lower')
        for i in range(len(trajectory) - 1, -1, -1):
            # plot the vehicle
            edge = trajectory[i]
            start_pose = edge.parent.pose.copy()
            color = 'b'
            for d in np.arange(0., edge.d, 0.2):
                c_point = edge.ptg.get_cpoint_at_d(d, edge.k)
                current_pose = start_pose + c_point.pose
                vehicle.phi = c_point.phi
                vehicle.plot(ax, current_pose, self.world, color)

                title = r'$x={0:.1f},y={1:.1f},\theta={2:+.1f}^\circ,\phi={3:+.1f}^\circ,\alpha={4:+.1f}^\circ$'.format(
                    current_pose.x,
                    current_pose.y,
                    deg(current_pose.theta),
                    deg(current_pose.phi),
                    deg(c_point.alpha))

                fig.suptitle(title)
                if goal is not None:
                    ax.plot(self.world.x_to_ix(goal.x), self.world.y_to_iy(goal.y), '+r')
                print('Saving frame {0}'.format(frame))
                plt.savefig('{0}{1:04d}.png'.format(file_name, frame))
                # clear the figure for next drawing
                ax.lines = []
                frame += 1

    @staticmethod
    def get_trajectory_edge(parent: Node, child: Node) -> Edge:
        for edge in parent.edges_to_child:
            if edge.end_pose == child.pose:
                return edge
