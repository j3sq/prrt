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
    plot_alphad(ptg)


def plot_alphad(ptg: PTG):
    fig, ax = plt.subplots(subplot_kw=dict(projection='polar'))
    for cpoints_at_k in ptg.cpoints:
        d = [cpoint.d for cpoint in cpoints_at_k]
        alpha = [cpoint.alpha for cpoint in cpoints_at_k]
        ax.plot(alpha, d, 'k')

    ax.annotate(r'$\alpha = {0:.1f}^\circ$'.format(30),
                xy=(rad(30), 5),  # theta, radius
                xytext=(rad(30), 6),  # fraction, fraction
                arrowprops=dict(facecolor='black', shrink=0.05),
                horizontalalignment='left',
                verticalalignment='bottom',
                fontsize=20)
    ax.annotate(r'$\alpha = {0:.1f}^\circ$'.format(15),
                xy=(rad(15), 5),  # theta, radius
                xytext=(rad(15), 6),  # fraction, fraction
                arrowprops=dict(facecolor='black', shrink=0.05),
                horizontalalignment='left',
                verticalalignment='bottom',
                fontsize=20)
    ax.annotate(r'$\alpha = {0:.1f}^\circ$'.format(0),
                xy=(rad(0), 5),  # theta, radius
                xytext=(rad(0), 6),  # fraction, fraction
                arrowprops=dict(facecolor='black', shrink=0.05),
                horizontalalignment='left',
                verticalalignment='bottom',
                fontsize=20)
    plt.show()


if __name__ == "__main__":
    main()
