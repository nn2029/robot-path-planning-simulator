"""A* search for a 4-connected occupancy grid."""

from __future__ import annotations

from heapq import heappop, heappush
from itertools import count

from .grid import Coord, GridMap, PlannerResult, manhattan, reconstruct_path


def astar(grid: GridMap, start: Coord, goal: Coord) -> PlannerResult:
    """Find a shortest grid path using A* with Manhattan distance.

    A* is usually the best first planner to show in a grid simulator because it
    combines Dijkstra's optimality with a heuristic that points the search at
    the goal. In production robots, the heuristic and edge costs often become
    richer: inflated costmaps, speed zones, terrain risk, and vehicle turn cost
    can all change what "shortest" means.
    """

    grid.validate_endpoint(start, "start")
    grid.validate_endpoint(goal, "goal")
    if start == goal:
        return PlannerResult.success_result([start], [start])

    open_heap: list[tuple[float, float, int, Coord]] = []
    tie_breaker = count()
    heappush(open_heap, (manhattan(start, goal), 0, next(tie_breaker), start))

    came_from: dict[Coord, Coord] = {}
    g_score: dict[Coord, float] = {start: 0}
    visited: set[Coord] = set()
    visited_order: list[Coord] = []

    while open_heap:
        _, current_cost, _, current = heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        visited_order.append(current)

        if current == goal:
            path = reconstruct_path(came_from, current)
            return PlannerResult.success_result(path, visited_order)

        for neighbor in grid.neighbors4(current):
            tentative_cost = current_cost + 1
            if tentative_cost >= g_score.get(neighbor, float("inf")):
                continue

            came_from[neighbor] = current
            g_score[neighbor] = tentative_cost
            priority = tentative_cost + manhattan(neighbor, goal)
            heappush(
                open_heap,
                (priority, tentative_cost, next(tie_breaker), neighbor),
            )

    return PlannerResult.failure(visited_order)

