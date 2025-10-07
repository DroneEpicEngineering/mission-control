def ned_to_enu(n: float, e: float, d: float) -> tuple[float, float, float]:
    return e, n, -d


def enu_to_ned(e: float, n: float, u: float) -> tuple[float, float, float]:
    return n, e, -u
