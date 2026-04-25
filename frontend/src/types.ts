export type Algorithm = "astar" | "dijkstra" | "rrt";

export type InteractionMode = "obstacle" | "erase" | "start" | "goal";

export interface Cell {
  x: number;
  y: number;
}

export interface PlannerInput {
  width: number;
  height: number;
  obstacles: Set<string>;
  start: Cell;
  goal: Cell;
}

export interface PlannerResult {
  path: Cell[];
  visited: Cell[];
  cost: number | null;
  success: boolean;
  message: string;
}

