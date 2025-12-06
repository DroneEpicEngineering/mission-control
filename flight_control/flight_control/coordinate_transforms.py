from typing_extensions import Self
from dataclasses import dataclass

import numpy as np


def ned_to_enu(n: float, e: float, d: float) -> tuple[float, float, float]:
    return float(e), float(n), float(-d)


def enu_to_ned(e: float, n: float, u: float) -> tuple[float, float, float]:
    return float(n), float(e), float(-u)


def heading_transform(heading: float) -> float:
    return -heading + (np.pi / 2.0)


@dataclass
class Coordinates:
    x: float
    y: float
    z: float


class SpatialVector:
    def __init__(self, x: float, y: float, z: float) -> None:
        # stores data in ENU convention
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def from_enu(cls, x: float, y: float, z: float) -> Self:
        return SpatialVector(x, y, z)

    @classmethod
    def from_enu_coords(cls, enu_coords: Coordinates) -> Self:
        return SpatialVector(enu_coords.x, enu_coords.y, enu_coords.z)

    @classmethod
    def from_ned(cls, x: float, y: float, z: float) -> Self:
        return SpatialVector(y, x, -z)

    @classmethod
    def from_ned_coords(cls, ned_coords: Coordinates) -> Self:
        return SpatialVector(ned_coords.y, ned_coords.x, -ned_coords.z)

    @classmethod
    def from_origin(cls) -> Self:
        return SpatialVector(0.0, 0.0, 0.0)

    def as_enu(self) -> tuple[float, float, float]:
        return self.x, self.y, self.z

    def as_enu_coords(self) -> Coordinates:
        return Coordinates(self.x, self.y, self.z)

    def as_ned(self) -> tuple[float, float, float]:
        return self.y, self.x, -self.z

    def as_ned_coords(self) -> Coordinates:
        return Coordinates(self.y, self.x, -self.z)
