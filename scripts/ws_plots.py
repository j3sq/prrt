from prrt.ptg import PTG, CPTG, AlphaA_PTG
from prrt.vehicle import ArticulatedVehicleFactory
import matplotlib.pyplot as plt
import yaml
from math import radians as rad, degrees as deg


def main():
    # Load configuration files
    vehicle_config_file = '../config/vehicle.yaml'
    aptg_config_file = '../config/FWD_CAPTG.yaml'
    with open(vehicle_config_file) as f:
        vehicle_config = yaml.load(f)
    with open(aptg_config_file) as f:
        aptg_config = yaml.load(f)

    # Overwrite configuration as needed
    vehicle_config['class_name'] = 'ArticulatedVehicle'
    vehicle_config['phi_max'] = 45.

    init_phi_deg = 0.
    aptg_config['init_phi'] = init_phi_deg
    aptg_config['grid_size'] = 5
    aptg_config['alpha_max'] = 165
    aptg_config['min_dist_between_cpoints'] = 0.1
    aptg_config['alpha_resolution'] = 15.

    # Build objects
    av = ArticulatedVehicleFactory.build_av(vehicle_config)
    ptg = AlphaA_PTG(av, aptg_config)
    ptg.build_cpoints()

    # Plot
    plot_trajectories(ptg)


def plot_trajectories(ptg: PTG):
    fig, ax = plt.subplots()
    for cpoints_at_k in ptg.cpoints:
        x = [cpoint.pose.x for cpoint in cpoints_at_k]
        y = [cpoint.pose.y for cpoint in cpoints_at_k]
        ax.plot(x, y, 'k')

    ax.title.set_text('Trajectories at $\phi_i = {0}^\circ$ '.format(deg(ptg.cpoints[0][0].phi)))
    k = ptg.alpha2idx(rad(0.))
    x = ptg.cpoints[k][10].x
    y = ptg.cpoints[k][10].y
    ax.annotate(r'$\alpha = {0:.1f}^\circ$'.format(0.),
                xy=(x, y),  # theta, radius
                xytext=(x-0.5, y-0.5),  # fraction, fraction
                arrowprops=dict(facecolor='black', shrink=0.05),
                horizontalalignment='left',
                verticalalignment='bottom',
                fontsize=20)
    k = ptg.alpha2idx(rad(15.))
    x = ptg.cpoints[k][-10].x
    y = ptg.cpoints[k][-10].y
    ax.annotate(r'$\alpha = {0:.1f}^\circ$'.format(15),
                xy=(x, y),  # theta, radius
                xytext=(x-0.5, y-0.5),  # fraction, fraction
                arrowprops=dict(facecolor='black', shrink=0.05),
                horizontalalignment='left',
                verticalalignment='bottom',
                fontsize=20)
    k = ptg.alpha2idx(rad(30))
    x = ptg.cpoints[k][-5].x
    y = ptg.cpoints[k][-5].y
    ax.annotate(r'$\alpha = {0:.1f}^\circ$'.format(30),
                xy=(x, y),  # theta, radius
                xytext=(x-0.5, y-0.5),  # fraction, fraction
                arrowprops=dict(facecolor='black', shrink=0.05),
                horizontalalignment='left',
                verticalalignment='bottom',
                fontsize=20)

    plt.show()


if __name__ == "__main__":
    main()
