import sys, traceback
import yaml
from prrt.vehicle import ArticulatedVehicle
from prrt.ptg import APTG
from math import radians as rad, degrees as deg
from prrt.helper import load_object


# How to run: python aptg_runner.py './path to vehicle config file' ''./path to aptg config file'

def main():
    try:
        # if no arguments were passed print help
        if len(sys.argv) == 1:
            print_help()
            return
        # process shared argument across all functions
        command = int(sys.argv[1])
        if command in [1, 2, 3, 4]:
            vehicle_config_file = sys.argv[2]
            aptg_config_file = sys.argv[3]
            with open(vehicle_config_file) as f:
                vehicle_config = yaml.load(f)
            av = ArticulatedVehicle(vehicle_config)
            with open(aptg_config_file) as f:
                aptg_config = yaml.load(f)
            aptg = APTG(av, aptg_config)
        elif command in [5]:
            aptg = load_object(sys.argv[2])
        else:
            print_help()
            return
        arg_count = len(sys.argv) - 2  # remove file name and command number

        # process commands
        if command == 1 and arg_count == 3:
            plot_ptg_cpoints(aptg, rad(float(sys.argv[4])))
        elif command == 2 and arg_count == 3:
            trace_trajectory_at_phi_alpha(aptg, rad(float(sys.argv[4])), rad(float(sys.argv[5])))
        elif command == 3 and arg_count == 3:
            trace_trajectory_at_phi(aptg, rad(float(sys.argv[4])))
        elif command == 4 and arg_count == 2:
            build_aptg(aptg)
        elif command == 5 and arg_count == 3:
            plot_ptg_obstacle_grid(aptg, rad(float(sys.argv[3])), rad(float(sys.argv[4])))
        else:
            print_help()
    except:
        print()
        print('Error! Make sure to follow usage guidelines shown below')
        print('Error details:')
        print(traceback.print_exc())
        print_help()


def plot_ptg_cpoints(aptg: APTG, init_phi: float):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots()
    aptg.build(skip_collision_calc=True)
    ptg = aptg.ptg_at_phi(init_phi)
    ax.title.set_text('Trajectories at $\phi_i = {0}^\circ$ '.format(deg(init_phi)))
    ptg.plot_trajectories(ax)
    plt.show()


def trace_trajectory_at_phi_alpha(aptg: APTG, init_phi: float, alpha: float):
    # plot the vehicle at each cpoint of the trajectory selected by fixed init_phi and fixed alpha
    import matplotlib.pyplot as plt
    name = 'trajectories_at_phi_{0:.0f}_alpha_{1:.0f}'.format(deg(init_phi), deg(alpha))
    aptg.build(skip_collision_calc=True)
    grid_size = aptg.vehicle.trailer_l * 4.
    ptg = aptg.ptg_at_phi(init_phi)
    cpoints_at_alpha = ptg.cpoints[ptg.alpha2idx(alpha)]
    fig, ax = plt.subplots()
    ax.set_xlim([-grid_size, grid_size])
    ax.set_ylim([-grid_size, grid_size])
    frame = -1
    for cpoint in cpoints_at_alpha:
        frame += 1

        aptg.vehicle.plot(ax, cpoint.pose, None)
        ax.title.set_text(
            r'Trajectory at $\phi_i = {0:.1f}^\circ, \alpha = {1:.1f}^\circ$ '.format(deg(init_phi), deg(alpha)))
        plt.savefig('./out/{0}_{1:04d}.png'.format(name, frame))
        ax.lines = []


def trace_trajectory_at_phi(aptg: APTG, init_phi: float):
    # same as trace_trajectory_at_phi_alpha, however all possible alpha values are considered
    import matplotlib.pyplot as plt
    name = 'trajectories_at_phi_{0:.0f}'.format(deg(init_phi))
    grid_size = aptg.vehicle.trailer_l * 4.
    aptg.build(skip_collision_calc=True)
    ptg = aptg.ptg_at_phi(init_phi)
    fig, ax = plt.subplots()
    ax.set_xlim([-grid_size, grid_size])
    ax.set_ylim([-grid_size, grid_size])
    frame = -1
    print('plotting trajectories, this will take a while!')
    for cpoints_at_alpha in ptg.cpoints:
        alpha = cpoints_at_alpha[0].alpha
        for cpoint in cpoints_at_alpha:
            frame += 1
            aptg.vehicle.plot(ax, cpoint.pose, None)
            ax.title.set_text(
                r'Trajectory at $\phi_i = {0:.1f}^\circ, \alpha = {1:.1f}^\circ$ '.format(deg(init_phi), deg(alpha)))
            plt.savefig('./out/{0}_{1:04d}.png'.format(name, frame))
            ax.lines = []


def plot_ptg_obstacle_grid(aptg: APTG, init_phi: float, alpha: float):
    import numpy as np
    import matplotlib.pyplot as plt
    ptg = aptg.ptg_at_phi(init_phi)
    k = ptg.alpha2idx(alpha)
    omap = -ptg.d_max * np.ones((ptg.obstacle_grid.cell_count_x, ptg.obstacle_grid.cell_count_y))
    fig, ax = plt.subplots()
    for x in range(ptg.obstacle_grid.cell_count_x):
        for y in range(ptg.obstacle_grid.cell_count_y):
            cell = ptg.obstacle_grid.cells[x, y]
            if cell is None:
                continue
            for kd_pair in cell:
                if kd_pair.k == k:
                    omap[y, x] = kd_pair.d
    ax.matshow(omap, origin='lower', cmap='RdYlGn')
    plt.show()


def build_aptg(aptg: APTG):
    print('building APTG, this will take a while!')
    aptg.build()
    aptg.dump('./{0}.pkl'.format(aptg.name))


def print_help():
    print()
    print('PTG Runner!')
    print('Usage:')
    print('Run: python aptg_runner.py [command number] [arg1] [arg2] .... ')
    print()
    print('Commands:')
    print('  1: Plot PTG cpoints')
    print('     Arguments:')
    print('       1: Vehicle configuration file')
    print('       2: APTG configuration file')
    print('       3: APTG initial articulation angle in deg')
    print('     Example: python aptg_runner.py 1 ./config/vehicle.yaml ./config/fwd_captg.yaml 20')
    print()
    print('  2: Trace vehicle trajectory at given initial phi(articulation angle) and alpha(steering angle).')
    print('     Arguments:')
    print('       1: Vehicle configuration file')
    print('       2: APTG configuration file')
    print('       3: Initial articulation angle(phi) in deg')
    print('       4: Steering angle(alpha) in deg')
    print('     Example: python aptg_runner.py 2 ./config/vehicle.yaml ./config/fwd_captg.yaml 0 15')
    print()
    print('  3: Trace vehicle trajectory at given initial phi(articulation angle) and all possible alpha values.')
    print('     Arguments:')
    print('       1: Vehicle configuration file')
    print('       2: APTG configuration file')
    print('       3: Initial articulation angle(phi) in deg')
    print('     Example: python aptg_runner.py 3 ./config/vehicle.yaml ./config/fwd_captg.yaml 0')
    print()
    print('  4: Build an APTG and save it to a file')
    print('     Arguments:')
    print('       1: Vehicle configuration file')
    print('       2: APTG configuration file')
    print('     Example: python aptg_runner.py 4 ./config/vehicle.yaml ./config/fwd_captg.yaml')
    print()
    print('  5: Plot ptg obstacle grid from a prebuilt APTG pickle file')
    print('     Arguments:')
    print('       1: APTG pickle file')
    print('       2: Initial articulation angle(phi) in deg')
    print('       3: Steering angle(alpha) in deg')
    print('     Example: python aptg_runner.py 5 ./jar/fwd_captg.pkl -30 15')


if __name__ == "__main__":
    main()
