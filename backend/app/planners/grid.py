"""Grid primitives shared by all planners.

This simulator models the world as a 4-connected occupancy grid. That is a
deliberately teachable abstraction: it makes A* and Dijkstra easy to inspect,
but it hides many production robotics concerns such as robot footprint
inflation, localization uncertainty, dynamic obstacles, terrain costmaps, and
vehicle kinematics.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import inf
from random import Random
from typing import Iterator, Mapping, Sequence, Tuple

Coord = Tuple[int, int]


@dataclass(frozen=True)
class PlannerResult:
    """Result envelope returned by every planner.

    ``visited`` is ordered by expansion time so the UI can animate or color the
    search frontier. ``cost`` is the number of grid moves for the grid planners;
    failed searches use ``inf`` to make "no route" explicit in tests.
    """

    path: list[Coord]
    visited: list[Coord]
    cost: float
    success: bool
    message: str

    @classmethod
    def success_result(cls, path: list[Coord], visited: list[Coord]) -> "PlannerResult":
        return cls(
            path=path,
            visited=visited,
            cost=max(0, len(path) - 1),
            success=True,
            message="Path found",
        )

    @classmethod
    def failure(cls, visited: list[Coord], message: str = "No path found") -> "PlannerResult":
        return cls(path=[], visited=visited, cost=inf, success=False, message=message)


@dataclass(frozen=True)
class GridMap:
    """A small immutable occupancy grid.

    Coordinates are ``(x, y)`` where ``x`` grows right and ``y`` grows down,
    matching canvas coordinates in the browser. Obstacles are stored as a
    ``frozenset`` so planner code can safely share maps without accidental
    mutation.
    """

    width: int
    height: int
    obstacles: frozenset[Coord] = field(default_factory=frozenset)

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Grid dimensions must be positive")

        obstacle_set = frozenset(self.obstacles)
        out_of_bounds = [cell for cell in obstacle_set if not self.in_bounds(cell)]
        if out_of_bounds:
            raise ValueError(f"Obstacle outside grid: {out_of_bounds[0]}")
        object.__setattr__(self, "obstacles", obstacle_set)

    @classmethod
    def from_rows(cls, rows: Sequence[str], obstacle: str = "#") -> "GridMap":
        """Create a map from text rows where ``#`` marks occupied cells."""

        if not rows:
            raise ValueError("At least one row is required")
        width = len(rows[0])
        if width == 0:
            raise ValueError("Rows cannot be empty")
        if any(len(row) != width for row in rows):
            raise ValueError("All rows must have the same width")

        obstacles = {
            (x, y)
            for y, row in enumerate(rows)
            for x, value in enumerate(row)
            if value == obstacle
        }
        return cls(width=width, height=len(rows), obstacles=frozenset(obstacles))

    def in_bounds(self, cell: Coord) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def traversable(self, cell: Coord) -> bool:
        return self.in_bounds(cell) and cell not in self.obstacles

    def validate_endpoint(self, cell: Coord, label: str) -> None:
        if not self.in_bounds(cell):
            raise ValueError(f"{label} is outside the grid")
        if cell in self.obstacles:
            raise ValueError(f"{label} is inside an obstacle")

    def neighbors4(self, cell: Coord) -> Iterator[Coord]:
        """Yield deterministic 4-connected neighbors.

        The order is stable on purpose. It makes unit tests reproducible and
        keeps tie-breaking from changing between Python versions.
        """

        x, y = cell
        for candidate in ((x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)):
            if self.traversable(candidate):
                yield candidate

    def free_cells(self) -> list[Coord]:
        return [
            (x, y)
            for y in range(self.height)
            for x in range(self.width)
            if (x, y) not in self.obstacles
        ]

    def random_free_cell(self, rng: Random) -> Coord:
        free = self.free_cells()
        if not free:
            raise ValueError("Grid contains no free cells")
        return free[rng.randrange(len(free))]


def manhattan(a: Coord, b: Coord) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def squared_distance(a: Coord, b: Coord) -> int:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def reconstruct_path(came_from: Mapping[Coord, Coord], current: Coord) -> list[Coord]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
