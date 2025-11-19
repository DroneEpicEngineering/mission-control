from dataclasses import dataclass


@dataclass
class NavigationInput:
    target_x: float
    target_y: float
    target_z: float

    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    psi: float

    dt: float


@dataclass
class NavigationOutput:
    x: float
    y: float
    z: float
    psi: float
