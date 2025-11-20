import numpy as np

from flight_control.navigation.types import NavigationInput


def calculate_los(data: NavigationInput) -> float:
    return np.arctan2(data.target_y - data.y, data.target_x - data.x)


def calculate_los_change(los: float, previous_los: float, time: float) -> float:
    return (np.arctan2(np.sin(los - previous_los), np.cos(los - previous_los))) / time


def calculate_distance(data: NavigationInput) -> float:
    return np.hypot(data.target_x - data.x, data.target_y - data.y)


def calculate_approach_velocity(
    distance: float, previous_distance: float, time: float
) -> float:
    return -(distance - previous_distance) / time
