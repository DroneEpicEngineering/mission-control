import numpy as np


def ned_to_enu(n: float, e: float, d: float) -> tuple[float, float, float]:
    return float(e), float(n), float(-d)


def enu_to_ned(e: float, n: float, u: float) -> tuple[float, float, float]:
    return float(n), float(e), float(-u)


def heading_transform(heading: float) -> float:
    return -heading + (np.pi / 2.0)
