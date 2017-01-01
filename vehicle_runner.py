import sys, traceback
import yaml
from prrt.vehicle import ArticulatedVehicle
from math import  radians as rad


def main():
    try:
        # if no arguments were passed print help
        if len(sys.argv) == 1:
            print_help()
            return
        # process shared argument across all functions
        command = int(sys.argv[1])
        vehicle_config_file = sys.argv[2]
        with open(vehicle_config_file) as f:
            vehicle_config = yaml.load(f)
        av = ArticulatedVehicle(vehicle_config)
        argv_count = len(sys.argv)

        # process commands
        if command == 1 and argv_count == 7:
            test_plot(av, float(sys.argv[3]), float(sys.argv[4]), rad(float(sys.argv[5])), rad(float(sys.argv[6])))
        # elif command ==2:
        #     # process command
        else:
            print_help()
    except:
        print()
        print('Error! Make sure to follow usage guidelines shown below')
        print('Error details:')
        print(traceback.print_exc())
        print_help()


def test_plot(av: ArticulatedVehicle, x: float, y: float, theta: float, phi: float):
    import matplotlib.pyplot as plt
    from prrt.primitive import PoseR2S2
    from math import radians as rad
    pose = PoseR2S2(x, y, theta, phi)
    grid_size = av.trailer_l * 4
    fig, ax = plt.subplots()
    ax.set_xlim([x - grid_size, x + grid_size])
    ax.set_ylim([y - grid_size, y + grid_size])
    av.plot(ax, pose, None)
    ax.title.set_text('Pose = {0} '.format(pose))
    plt.show()


def print_help():
    print()
    print('Vehicle Runner!')
    print('Usage:')
    print('Run: python vehicle_runner.py [command number] [arg1] [arg2] .... ')
    print()
    print('Commands:')
    print('  1: Test the vehicle shape by plotting the vehicle at a given pose')
    print('     Arguments:')
    print('       1: Vehicle configuration file')
    print('       2: pose x position')
    print('       3: pose y position')
    print('       4: pose heading angle in deg')
    print('       5: pose articulation angle in deg')
    print('     Example: python vehicle_runner.py 1 ./config/vehicle.yaml 10 5 30 -15')


if __name__ == "__main__":
    main()
