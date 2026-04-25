"""A small RRT-style planner adapted to grid cells."""

from __future__ import annotations

from dataclasses import dataclass
from random import Random

from .grid import Coord, GridMap, PlannerResult, reconstruct_path, squared_distance


@dataclass(frozen=True)
class GridRRTPlanner:
    """Rapidly-exploring Random Tree style planner for the simulator grid.

    Classical RRT samples continuous configuration space and steers toward
    random states while checking collisions along edges. This educational
    version keeps the same "sample, nearest, extend" shape but expands one
    4-connected grid cell at a time. It is not optimal, and unlike A* or
    Dijkstra it can miss a path within the iteration budget. That tradeoff is
    exactly why it is useful to show next to deterministic graph search.
    """

    max_iterations: int = 900
    goal_sample_rate: float = 0.2
    seed: int = 7

    def plan(self, grid: GridMap, start: Coord, goal: Coord) -> PlannerResult:
        grid.validate_endpoint(start, "start")
        grid.validate_endpoint(goal, "goal")
        if start == goal:
            return PlannerResult.success_result([start], [start])

        rng = Random(self.seed)
        tree_parent: dict[Coord, Coord] = {}
        nodes: list[Coord] = [start]
        visited_order: list[Coord] = [start]

        for _ in range(self.max_iterations):
            sample = goal if rng.random() < self.goal_sample_rate else grid.random_free_cell(rng)
            nearest = min(nodes, key=lambda node: squared_distance(node, sample))
            new_node = self._steer_one_cell(grid, nearest, sample, set(nodes))
            if new_node is None:
                continue

            tree_parent[new_node] = nearest
            nodes.append(new_node)
            visited_order.append(new_node)

            if new_node == goal:
                path = reconstruct_path(tree_parent, new_node)
                return PlannerResult.success_result(path, visited_order)

        return PlannerResult.failure(
            visited_order,
            message="RRT did not connect start to goal within the iteration budget",
        )

    @staticmethod
    def _steer_one_cell(
        grid: GridMap,
        nearest: Coord,
        sample: Coord,
        existing_nodes: set[Coord],
    ) -> Coord | None:
        """Extend from ``nearest`` by one free grid cell toward ``sample``.

        On a real robot this step would integrate the vehicle model and check
        the whole segment for collision. The grid version moves one cell so the
        behavior stays visible in the UI.
        """

        candidates = [cell for cell in grid.neighbors4(nearest) if cell not in existing_nodes]
        if not candidates:
            return None
        candidates.sort(key=lambda cell: (squared_distance(cell, sample), cell[1], cell[0]))
        return candidates[0]
