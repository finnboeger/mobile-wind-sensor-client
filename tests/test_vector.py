import math

from utils.vector import PolarCoordinates, Vector2D


def test_polar_round_trip() -> None:
    vector = Vector2D.from_polar(PolarCoordinates(angle=math.radians(90), magnitude=2))
    assert abs(vector.x) < 1e-9
    assert abs(vector.y - 2) < 1e-9

    polar = vector.to_polar()
    assert abs(polar.angle - math.radians(90)) < 1e-9
    assert abs(polar.magnitude - 2) < 1e-9


def test_angle_is_normalized_to_positive_range() -> None:
    polar = Vector2D(x=0, y=-1).to_polar()
    assert abs(polar.angle - math.radians(270)) < 1e-9
