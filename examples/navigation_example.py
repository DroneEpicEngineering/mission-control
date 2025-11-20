import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from flight_control.navigation import algorithms as algs
from flight_control.navigation import NavigationContext
from flight_control.navigation.types import NavigationInput

DELTA_TIME = 0.02


# strategy = algs.TrueProportionalNavigation(N=4, Vd=2)
strategy = algs.FastResponseProportionalNavigation(G=3, W=0.5)
context = NavigationContext(strategy)

target_data = pd.read_csv("data/test_trajectory.csv")
uav_data = (0, 0, 0, 0)

target_trajectory = []
uav_trajectory = []

for index, row in target_data.iterrows():
    target_trajectory.append((row["pos_x"], row["pos_y"], row["pos_z"]))
    uav_trajectory.append(uav_data)

    data = NavigationInput(
        target_x=row["pos_x"],
        target_y=row["pos_y"],
        target_z=row["pos_z"],
        x=uav_data[0],
        y=uav_data[1],
        z=uav_data[2],
        psi=uav_data[3],
        dt=DELTA_TIME,
    )
    result = context.execute(data)
    uav_data = (result.x, result.y, result.z, result.psi)

    if abs(result.x - row["pos_x"]) < 1 and abs(result.y - row["pos_y"]) < 1:
        break

uav_trajectory.append(uav_data)
target_trajectory = np.array(target_trajectory)
uav_trajectory = np.array(uav_trajectory)

plt.figure(figsize=(12, 8))
plt.plot(
    target_trajectory[:, 0],
    target_trajectory[:, 1],
    "r-",
    label="Target Trajectory",
    linewidth=2,
)
plt.plot(
    uav_trajectory[:, 0],
    uav_trajectory[:, 1],
    "b-",
    label="UAV Trajectory",
    linewidth=2,
)

plt.plot(
    target_trajectory[0, 0],
    target_trajectory[0, 1],
    "go",
    markersize=8,
    label="Target Start",
)
plt.plot(
    target_trajectory[-1, 0],
    target_trajectory[-1, 1],
    "ro",
    markersize=8,
    label="Target End",
)
plt.plot(
    uav_trajectory[0, 0], uav_trajectory[0, 1], "g^", markersize=8, label="UAV Start"
)
plt.plot(
    uav_trajectory[-1, 0], uav_trajectory[-1, 1], "b^", markersize=8, label="UAV End"
)

plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Target vs UAV Trajectory (2D)")
plt.legend()
plt.grid(True, alpha=0.3)
plt.axis("equal")
plt.show()
