import sys, traceback
import yaml
from prrt.planner import Planner


def main():
    try:
        # process shared argument across all functions
        command = int(sys.argv[1])
        if command in [1]:
            planner_config_file = sys.argv[2]
            with open(planner_config_file) as f:
                planner_config = yaml.load(f)
            planner = Planner(planner_config)
        else:
            print_help()
            return
        arg_count = len(sys.argv) - 2  # remove file name and command number

        # process commands
        if command == 1 and arg_count == 1:
            solve(planner)

        else:
            print_help()
    except:
        print()
        print('Error! Make sure to follow usage guidelines shown below')
        print('Error details:')
        print(traceback.print_exc())
        print_help()

    planner_config_file = yaml.load(sys.argv[1])
    with open(planner_config_file) as f:
        config = yaml.load(f)


def solve(planner: Planner):
    planner.solve()
    planner.tree.plot_nodes(planner.world)
    planner.trace_solution(planner.aptgs[0].vehicle)


def print_help():
    print()
    print('Planner Runner!')
    print('Usage:')
    print('Run: python planner_runner.py [command number] [arg1] [arg2] .... ')
    print()
    print('Commands:')
    print('  1: Solve using RRT')
    print('     Arguments:')
    print('       1: planner configuration file')
    print('     Example: python planner_runner.py 1 ./config/planner.yaml')


if __name__ == "__main__":
    main()
