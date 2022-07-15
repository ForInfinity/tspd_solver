TRUCK_SPEED = 30  # km/h
TRUCK_SPEED_MPS = TRUCK_SPEED / 3.6

DRONE_SPEED = 40  # km/h
DRONE_SPEED_MPS = DRONE_SPEED / 3.6


def calc_truck_time(distance: float) -> float:
    global TRUCK_SPEED_MPS
    return distance / TRUCK_SPEED_MPS


def calc_drone_time(distance: float) -> float:
    global DRONE_SPEED_MPS
    return distance / DRONE_SPEED_MPS
