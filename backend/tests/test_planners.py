import unittest

from app.planners import GridMap, GridRRTPlanner, astar, dijkstra


class PlannerCorrectnessTests(unittest.TestCase):
    def setUp(self) -> None:
        self.grid = GridMap.from_rows(
            [
                "..#...",
                "..#...",
                "..#...",
                "..#...",
                "......",
            ]
        )
        self.start = (0, 0)
        self.goal = (5, 4)

    def assert_valid_path(self, path):
        self.assertGreater(len(path), 0)
        self.assertEqual(path[0], self.start)
        self.assertEqual(path[-1], self.goal)

        for cell in path:
            self.assertTrue(self.grid.traversable(cell), f"{cell} is not traversable")

        for before, after in zip(path, path[1:]):
            distance = abs(before[0] - after[0]) + abs(before[1] - after[1])
            self.assertEqual(distance, 1, f"{before} -> {after} is not 4-connected")

    def test_astar_finds_shortest_path_around_wall_gap(self):
        result = astar(self.grid, self.start, self.goal)

        self.assertTrue(result.success)
        self.assertEqual(result.cost, 9)
        self.assert_valid_path(result.path)
        self.assertIn(self.start, result.visited)

    def test_dijkstra_matches_astar_shortest_cost(self):
        astar_result = astar(self.grid, self.start, self.goal)
        dijkstra_result = dijkstra(self.grid, self.start, self.goal)

        self.assertTrue(dijkstra_result.success)
        self.assertEqual(dijkstra_result.cost, astar_result.cost)
        self.assert_valid_path(dijkstra_result.path)
        self.assertGreaterEqual(len(dijkstra_result.visited), len(astar_result.visited))

    def test_unreachable_goal_returns_failure_without_path(self):
        boxed_grid = GridMap(width=3, height=3, obstacles=frozenset({(1, 0), (0, 1), (1, 1)}))

        result = astar(boxed_grid, (0, 0), (2, 2))

        self.assertFalse(result.success)
        self.assertEqual(result.path, [])
        self.assertEqual(result.message, "No path found")

    def test_rrt_style_planner_is_deterministic_with_goal_bias(self):
        planner = GridRRTPlanner(max_iterations=80, goal_sample_rate=1.0, seed=3)

        result = planner.plan(self.grid, self.start, self.goal)

        self.assertTrue(result.success)
        self.assertEqual(result.cost, 9)
        self.assert_valid_path(result.path)

    def test_start_or_goal_inside_obstacle_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "start is inside an obstacle"):
            astar(self.grid, (2, 0), self.goal)

    def test_expansion_limit_stops_grid_search(self):
        result = astar(self.grid, self.start, self.goal, max_expansions=2)

        self.assertFalse(result.success)
        self.assertEqual(result.message, "Search expansion limit reached")
        self.assertGreaterEqual(len(result.visited), 2)


if __name__ == "__main__":
    unittest.main()
