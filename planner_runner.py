import sys, traceback
import yaml
from prrt.planner import Planner


def main():
    try:
        # if no arguments were passed print help
        if len(sys.argv) == 1:
            print_help()
            return
        # process shared argument across all functions
        n_iterations = 1
        command = int(sys.argv[1])
        if command in [1, 2]:
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
        elif command == 2 and arg_count == 2:
            n_iterations = int(sys.argv[3])
            print("The planner will run ", n_iterations, " times, to build statistics, this will take a while !")
            #create the file for the statistics and add an header to the first line
            import csv
            with open('stats.csv', 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile, delimiter=',')
                csv_writer.writerow(['success','number_of_iterations', 'total_number_of_nodes', 'best_path_length', 'best_distance_to_target'])
            # starting iterations
            for i in range(0, n_iterations):
                print ("Solving iteration number ", i)
                solve(planner)
                success, number_of_iterations, total_number_of_nodes, best_path_length, best_distance_to_target = planner.getResults()
                print("results:")
                print("       1. Solve success ", success)
                print("       2. Number of iterations ", number_of_iterations)
                print("       3. Total number of nodes in the tree ", total_number_of_nodes)
                print("       4. Best path length ", best_path_length)
                print("       5. Best distance to target ", best_distance_to_target)
                statistics_to_csv('stats.csv',
                                  success,
                                  number_of_iterations,
                                  total_number_of_nodes,
                                  best_path_length,
                                  best_distance_to_target)
            print("Finish, building statistics")
        else:
            print_help()
    except:
        print()
        print('Error! Make sure to follow usage guidelines shown below')
        print('Error details:')
        print(traceback.print_exc())
        print_help()



def solve(planner: Planner):
    planner.solve()

def statistics_to_csv(file_name='stats.csv',
                      success=False,
                      number_of_iterations=0,
                      total_number_of_nodes=0,
                      best_path_length=0.0,
                      best_distance_to_target=0.0):
    import csv
    with open(file_name, 'a', newline='') as csvfile:
        csv_writer = csv.writer(csvfile, delimiter=',')
        row = [success, number_of_iterations, total_number_of_nodes, best_path_length, best_distance_to_target]
        csv_writer.writerow( ['{0:+.4f}'.format(x) for x in row])



    print('Dumping solution to csv file done')

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
    print('  2: Solve n-times using RRT and build statistics ')
    print('     Arguments:')
    print('       1: planner configuration file')
    print('       2: number of iterations')
    print('     Example: python planner_runner.py 1 ./config/planner.yaml 100')

if __name__ == "__main__":
    main()
