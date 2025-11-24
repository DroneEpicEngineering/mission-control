from dataclasses import dataclass


@dataclass
class Odometry:
    x: float
    y: float
    z: float

    vx: float
    vy: float
    vz: float

    psi: float

    @property
    def position(self) -> tuple[float, float, float]:
        return float(self.x), float(self.y), float(self.z)

    @property
    def velocity(self) -> tuple[float, float, float]:
        return float(self.vx), float(self.vy), float(self.vz)


@dataclass
class NavigationInput:
    target_odom: Odometry
    uav_odom: Odometry

    dt: float


@dataclass
class NavigationOutput:
    ax: float
    ay: float
    az: float
    psi: float


@dataclass
class NavigationState:
    los: float
    R: float
