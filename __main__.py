from prrt.vehicle import Car, ArticulatedVehicle
from prrt.primitive import PointR2, PoseR2S1
from prrt.ptg import CPTG, ACPTG
import numpy as np
import prrt.helper as helper
from prrt.planner import Planner
import os
import time

# configurations
rebuild_tables = False
plot_tree = True
plot_trajectory_frames = True

car = Car(2., np.deg2rad(60), 2., 2.)
car_vertices = (PointR2(-1.5, -1.), PointR2(1.5, -1.), PointR2(1.5, 1.), PointR2(-1.5, 1))
car.set_vertices(car_vertices)

av = ArticulatedVehicle(2., np.deg2rad(60), 2., 2., 2., 1., 1., 2., 3.)
av.phi = 0

if rebuild_tables or not os.path.isfile('./prrt.pkl'):
    ptgs = []  # type: List[PTG]
    # fwd_circular_ptg = CPTG(5.0, car, 0.1, 1)
    for phi in np.arange(np.deg2rad(-45), np.deg2rad(45) + 0.01, np.deg2rad(3)):
        fwd_circular_ptg = ACPTG(5.0, av, 0.1, 1, phi)
        fwd_circular_ptg.name = 'Forward ACPTG @phi = {0:.0f}'.format(np.rad2deg(phi))
        fwd_circular_ptg.build_cpoints()
        fwd_circular_ptg.build_cpoints_grid()
        fwd_circular_ptg.build_obstacle_grid()
        # fwd_circular_ptg.plot_cpoints()
        ptgs.append(fwd_circular_ptg)

    # bwd_circular_ptg = CPTG(5.0, car, 0.1, -1)
    # bwd_circular_ptg.name = 'Backward Circular PTG'
    # bwd_circular_ptg.build_cpoints()
    # bwd_circular_ptg.build_cpoints_grid()
    # bwd_circular_ptg.build_obstacle_grid()
    # ptgs.append(bwd_circular_ptg)

    planner = Planner(ptgs)
    planner.load_world_map('./lot_caseStudy.png', 117.6, 68.3)  # this should not be done here !
    helper.save_object(planner, './prrt.pkl')
else:
    planner = helper.load_object('./prrt.pkl')  # type: Planner
    # planner.load_world_map('./lot.png', 117.6, 68.3)  # this should not be done here !
# Set initial pose and goal pose
init_pose = PoseR2S1(80, 60, 0.0 * np.pi)
goal_pose = PoseR2S1(55, 10, -3. / 4 * np.pi)

print(" SOLVING ")
start = time.time()
planner.solve_av(init_pose, goal_pose, 7.)
end = time.time()
print("time elapsed in s = ")
print(end - start)

# Plot as configured
if plot_tree:
    planner.tree.plot_nodes(planner.world, goal_pose)
if plot_trajectory_frames:
    # delete existing frames (if any)
    file_list = [f for f in os.listdir('./out') if f.endswith('.png') and f.startswith('frame')]
    for f in file_list:
        os.remove('./out/' + f)
    planner.trace_solution(av, goal_pose)
