"""Dijkstra search for a 4-connected occupancy grid."""

from __future__ import annotations

from heapq import heappop, heappush
from itertools import count

from .grid import Coord, GridMap, PlannerResult, reconstruct_path


def dijkstra(
    grid: GridMap,
    start: Coord,
    goal: Coord,
    max_expansions: int | None = None,
) -> PlannerResult:
    """Find a shortest grid path without a goal-directed heuristic.

    Dijkstra is a useful baseline because it is optimal with non-negative edge
    costs and makes no assumptions about the goal direction. The tradeoff is
    expansion volume: on open maps it explores much more of the grid than A*.
    That makes it a good teaching contrast in the simulator stats panel.
    """

    grid.validate_endpoint(start, "start")
    grid.validate_endpoint(goal, "goal")
    if start == goal:
        return PlannerResult.success_result([start], [start])

    open_heap: list[tuple[float, int, Coord]] = []
    tie_breaker = count()
    heappush(open_heap, (0, next(tie_breaker), start))

    came_from: dict[Coord, Coord] = {}
    distance: dict[Coord, float] = {start: 0}
    visited: set[Coord] = set()
    visited_order: list[Coord] = []

    while open_heap:
        current_cost, _, current = heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        visited_order.append(current)
        if max_expansions is not None and len(visited_order) > max_expansions:
            return PlannerResult.failure(
                visited_order,
                message="Search expansion limit reached",
            )

        if current == goal:
            path = reconstruct_path(came_from, current)
            return PlannerResult.success_result(path, visited_order)

        for neighbor in grid.neighbors4(current):
            candidate_cost = current_cost + 1
            if candidate_cost >= distance.get(neighbor, float("inf")):
                continue

            distance[neighbor] = candidate_cost
            came_from[neighbor] = current
            heappush(open_heap, (candidate_cost, next(tie_breaker), neighbor))

    return PlannerResult.failure(visited_order)
