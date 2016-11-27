import pickle
import numpy as np

INT_MAX = 2147483647
INT_MIN = -2147483648


def wrap_to_npi_pi(theta: float):
    return (theta + np.pi) % (2. * np.pi) - np.pi


def wrap_to_0_2pi(theta: float) -> float:
    neg = theta < 0
    theta %= 2. * np.pi
    if neg:
        theta += 2. * np.pi
    return theta


def angle_distance(ang_from: float, ang_to: float) -> float:
    ang_from = wrap_to_npi_pi(ang_from)
    ang_to = wrap_to_npi_pi(ang_to)
    d = ang_to - ang_from
    return wrap_to_npi_pi(d)


def save_object(obj, filename):
    with open(filename, 'wb') as output:
        pickle.dump(obj, output, pickle.HIGHEST_PROTOCOL)


def load_object(filename):
    with open(filename, 'rb') as input_file:
        return pickle.load(input_file)
