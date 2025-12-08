import numpy as np

from flight_control.navigation.types import NavigationInput


def calculate_los(data: NavigationInput) -> float:
    return np.arctan2(
        data.target_odom.y - data.uav_odom.y, data.target_odom.x - data.uav_odom.x
    )


def calculate_los_change(los: float, previous_los: float, time: float) -> float:
    return (np.arctan2(np.sin(los - previous_los), np.cos(los - previous_los))) / time


def calculate_distance(data: NavigationInput) -> float:
    return np.hypot(
        data.target_odom.x - data.uav_odom.x, data.target_odom.y - data.uav_odom.y
    )


def calculate_approach_velocity(
    distance: float, previous_distance: float, time: float
) -> float:
    return -(distance - previous_distance) / time
