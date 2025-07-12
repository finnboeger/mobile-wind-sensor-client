import math
from dataclasses import dataclass


@dataclass(frozen=True, kw_only=True)
class PolarCoordinates:
    """
    A 2D vector represented by angle and magnitude.

    The angle is in radians, and the magnitude is unit-less.
    """

    angle: float
    magnitude: float


@dataclass(frozen=True, kw_only=True)
class Vector2D:
    """
    A 2D vector represented by its cartesian coordinates.

    The x and y coordinates are unit-less.
    """

    x: float
    y: float

    @staticmethod
    def from_polar(coordinates: PolarCoordinates) -> "Vector2D":
        """
        Convert polar coordinates to a 2D vector.

        :param coordinates: A PolarCoordinates object
        :return: A Vector2D object
        """
        x = coordinates.magnitude * math.cos(coordinates.angle)
        y = coordinates.magnitude * math.sin(coordinates.angle)
        return Vector2D(x=x, y=y)

    def to_polar(self) -> PolarCoordinates:
        """
        Convert the 2D vector to polar coordinates.

        :return: A PolarCoordinates object
        """
        angle = math.atan2(self.y, self.x) % math.tau
        magnitude = math.sqrt(self.x**2 + self.y**2)
        return PolarCoordinates(angle=angle, magnitude=magnitude)

    def __repr__(self) -> str:
        """
        Represent the Vector2D object as a string.

        :return: A string representation of the vector
        """
        return f"Vector2D(x={self.x}, y={self.y})"

    def __add__(self, other: "Vector2D") -> "Vector2D":
        """
        Add two 2D vectors.

        :param other: Another Vector2D object
        :return: A new Vector2D object representing the sum
        """
        return Vector2D(x=self.x + other.x, y=self.y + other.y)

    def __sub__(self, other: "Vector2D") -> "Vector2D":
        """
        Subtract two 2D vectors.

        :param other: Another Vector2D object
        :return: A new Vector2D object representing the difference
        """
        return Vector2D(x=self.x - other.x, y=self.y - other.y)

    def __truediv__(self, other: float) -> "Vector2D":
        """
        Divide the vector by a scalar.

        :param other: A scalar value
        :return: A new Vector2D object representing the scaled vector
        """
        if other == 0:
            error_msg = "Cannot divide by zero"
            raise ZeroDivisionError(error_msg)
        return Vector2D(x=self.x / other, y=self.y / other)
