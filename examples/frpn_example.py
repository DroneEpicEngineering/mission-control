import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

from flight_control.navigation import algorithms as algs
from flight_control.navigation import NavigationContext
from flight_control.navigation.types import NavigationInput, Odometry

DELTA_TIME = 0.2

strategy = algs.FastResponseProportionalNavigation(G=19.7, W=0.051, a_max=0.2)
context = NavigationContext(strategy)

target_data = pd.read_csv("data/test_trajectory.csv")
uav_data = Odometry(0, 0, 0, 0, 0, 0, 0)

target_trajectory = []
uav_trajectory = []

for _, row in target_data.iterrows():
    target_data = Odometry(
        x=row["pos_x"],
        y=row["pos_y"],
        z=row["pos_z"],
        vx=row["vel_x"],
        vy=row["vel_y"],
        vz=row["vel_z"],
        psi=0.0,
    )

    data = NavigationInput(target_odom=target_data, uav_odom=uav_data, dt=DELTA_TIME)
    result = context.execute(data)

    current_position = np.array(uav_data.position)
    current_velocity = np.array(uav_data.velocity)
    acceleration = np.array([result.ax, result.ay, result.az])

    new_velocity = current_velocity + (acceleration * DELTA_TIME)
    new_position = current_position + (current_velocity * DELTA_TIME)

    uav_data = Odometry(*new_position, *new_velocity, psi=float(result.psi))

    target_trajectory.append(target_data)
    uav_trajectory.append(uav_data)


# Convert to numpy arrays for easier manipulation
target_trajectory = np.array(target_trajectory)
uav_trajectory = np.array(uav_trajectory)

# Create the figure and 3D axis
fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(111, projection="3d")

# Create buttons area
plt.subplots_adjust(bottom=0.2)

# Button axes
ax_prev = plt.axes([0.1, 0.05, 0.1, 0.075])
ax_next = plt.axes([0.21, 0.05, 0.1, 0.075])
ax_play = plt.axes([0.32, 0.05, 0.1, 0.075])
ax_pause = plt.axes([0.43, 0.05, 0.1, 0.075])
ax_reset = plt.axes([0.54, 0.05, 0.1, 0.075])

# Create buttons
btn_prev = Button(ax_prev, "Previous")
btn_next = Button(ax_next, "Next")
btn_play = Button(ax_play, "Play")
btn_pause = Button(ax_pause, "Pause")
btn_reset = Button(ax_reset, "Reset")

# Current frame variable
current_frame = 0
animation_running = False
animation = None


def set_equal_aspect_ratio():
    """Set equal aspect ratio for 3D plot"""
    limit = 30
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)


def update_display(frame):
    """Update the display for a given frame"""
    ax.clear()

    ax.plot(
        [odom.position[0] for odom in target_trajectory[: frame + 1]],
        [odom.position[1] for odom in target_trajectory[: frame + 1]],
        [odom.position[2] for odom in target_trajectory[: frame + 1]],
        "b-",
        label="Target Trajectory",
        alpha=0.7,
        linewidth=2,
    )
    ax.plot(
        [odom.position[0] for odom in uav_trajectory[: frame + 1]],
        [odom.position[1] for odom in uav_trajectory[: frame + 1]],
        [odom.position[2] for odom in uav_trajectory[: frame + 1]],
        "r-",
        label="UAV Trajectory",
        alpha=0.7,
        linewidth=2,
    )

    ax.scatter(
        *target_trajectory[frame].position, c="blue", s=100, label="Target", marker="o"
    )
    ax.scatter(*uav_trajectory[frame].position, c="red", s=100, label="UAV", marker="^")

    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_zlabel("Z Position")

    set_equal_aspect_ratio()
    ax.legend()


def next_frame(event):
    """Go to next frame"""
    global current_frame
    if current_frame < len(target_trajectory) - 1:
        current_frame += 1
        update_display(current_frame)
        plt.draw()


def prev_frame(event):
    """Go to previous frame"""
    global current_frame
    if current_frame > 0:
        current_frame -= 1
        update_display(current_frame)
        plt.draw()


def play_animation(event):
    """Start automatic animation"""
    global animation_running, animation
    if not animation_running:
        animation_running = True
        animation = FuncAnimation(
            fig,
            auto_animation_update,
            frames=range(current_frame, len(target_trajectory)),
            interval=100,
            repeat=False,
            blit=False,
        )
        plt.draw()


def pause_animation(event):
    """Pause automatic animation"""
    global animation_running, animation
    if animation_running and animation is not None:
        animation_running = False
        animation.event_source.stop()
        plt.draw()


def reset_animation(event):
    """Reset to first frame"""
    global current_frame, animation_running, animation
    if animation_running and animation is not None:
        animation.event_source.stop()
        animation_running = False
    current_frame = 0
    update_display(current_frame)
    plt.draw()


def auto_animation_update(frame):
    """Update function for automatic animation"""
    global current_frame
    current_frame = frame
    update_display(frame)
    if frame == len(target_trajectory) - 1:
        global animation_running
        animation_running = False


btn_next.on_clicked(next_frame)
btn_prev.on_clicked(prev_frame)
btn_play.on_clicked(play_animation)
btn_pause.on_clicked(pause_animation)
btn_reset.on_clicked(reset_animation)


def on_key(event):
    """Handle keyboard events"""
    global current_frame, animation_running, animation
    if event.key == "right":
        next_frame(None)
    elif event.key == "left":
        prev_frame(None)
    elif event.key == " ":
        if animation_running:
            pause_animation(None)
        else:
            play_animation(None)
    elif event.key == "r":
        reset_animation(None)


fig.canvas.mpl_connect("key_press_event", on_key)

update_display(current_frame)

instruction_text = "Controls: ← Previous  → Next  Space: Play/Pause  R: Reset"
plt.figtext(
    0.5,
    0.01,
    instruction_text,
    ha="center",
    fontsize=10,
    bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray"),
)

plt.show()
