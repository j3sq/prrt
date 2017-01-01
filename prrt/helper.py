import pickle
from math import fmod, pi as PI

INT_MAX = 2147483647
INT_MIN = -2147483648


def wrap_to_npi_pi(theta: float):
    return (theta + PI) % (2. * PI) - PI


def wrap_to_0_2pi(theta: float) -> float:
    neg = theta < 0
    theta = fmod(theta, 2. * PI)
    if neg:
        theta += 2. * PI
    return theta


def angle_distance(ang_from: float, ang_to: float) -> float:
    ang_from = wrap_to_npi_pi(ang_from)
    ang_to = wrap_to_npi_pi(ang_to)
    d = ang_to - ang_from
    return wrap_to_npi_pi(d)


def save_object(obj, file_name):
    with open(file_name, 'wb') as output:
        pickle.dump(obj, output, pickle.HIGHEST_PROTOCOL)


def load_object(filename):
    with open(filename, 'rb') as input_file:
        return pickle.load(input_file)
