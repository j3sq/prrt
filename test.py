from prrt.vehicle import ArticulatedVehicle, Vehicle
import numpy as np
import matplotlib.pyplot as plt
from prrt.primitive import PoseR2S1


def plot_vehicle(vehicle: Vehicle, pose: PoseR2S1, phi: float):
    av.phi = phi
    fig, ax = plt.subplots()
    vertices = vehicle.get_vertices_at_pose(pose)
    ax.plot(-30, -30)
    ax.plot(30, 30)
    for j in range(len(vertices) - 1):
        a = vertices[j]
        b = vertices[j + 1]
        ax.plot([a.x, b.x], [a.y, b.y], 'b')

    a = vertices[3]
    b = vertices[0]
    ax.plot([a.x, b.x], [a.y, b.y], 'b')
    a = vertices[9]
    b = vertices[6]
    ax.plot([a.x, b.x], [a.y, b.y], 'b')
    plt.show()


if __name__ == "__main__":
    av = ArticulatedVehicle(2., np.deg2rad(60), 2., 2., 2., 2., 1., 2, 10.)
    plot_vehicle(av, PoseR2S1(0., 0., 0.), 0)
    plot_vehicle(av, PoseR2S1(15., 0., 0.), np.deg2rad(60))
    plot_vehicle(av, PoseR2S1(15., 0., np.deg2rad(0)), np.deg2rad(-60))
