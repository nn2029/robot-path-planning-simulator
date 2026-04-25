import type { Algorithm, Cell, PlannerInput, PlannerResult } from "./types";

interface ScoredCell extends Cell {
  priority: number;
  cost: number;
}

const directions: Cell[] = [
  { x: 1, y: 0 },
  { x: 0, y: 1 },
  { x: -1, y: 0 },
  { x: 0, y: -1 },
];

export function cellKey(cell: Cell): string {
  return `${cell.x},${cell.y}`;
}

export function sameCell(a: Cell, b: Cell): boolean {
  return a.x === b.x && a.y === b.y;
}

export function runPlanner(algorithm: Algorithm, input: PlannerInput): PlannerResult {
  if (algorithm === "astar") {
    return astar(input);
  }
  if (algorithm === "dijkstra") {
    return dijkstra(input);
  }
  return rrt(input);
}

function astar(input: PlannerInput): PlannerResult {
  if (sameCell(input.start, input.goal)) {
    return success([input.start], [input.start]);
  }

  const open: ScoredCell[] = [
    { ...input.start, priority: manhattan(input.start, input.goal), cost: 0 },
  ];
  const cameFrom = new Map<string, string>();
  const costSoFar = new Map<string, number>([[cellKey(input.start), 0]]);
  const visited = new Set<string>();
  const visitedOrder: Cell[] = [];

  while (open.length > 0) {
    open.sort((a, b) => a.priority - b.priority || a.cost - b.cost || a.y - b.y || a.x - b.x);
    const current = open.shift()!;
    const currentKey = cellKey(current);
    if (visited.has(currentKey)) {
      continue;
    }
    visited.add(currentKey);
    visitedOrder.push({ x: current.x, y: current.y });

    if (sameCell(current, input.goal)) {
      return success(reconstructPath(cameFrom, currentKey), visitedOrder);
    }

    for (const neighbor of neighbors4(input, current)) {
      const newCost = current.cost + 1;
      const neighborKey = cellKey(neighbor);
      if (newCost >= (costSoFar.get(neighborKey) ?? Number.POSITIVE_INFINITY)) {
        continue;
      }
      cameFrom.set(neighborKey, currentKey);
      costSoFar.set(neighborKey, newCost);
      open.push({
        ...neighbor,
        cost: newCost,
        priority: newCost + manhattan(neighbor, input.goal),
      });
    }
  }

  return failure(visitedOrder, "No path found");
}

function dijkstra(input: PlannerInput): PlannerResult {
  if (sameCell(input.start, input.goal)) {
    return success([input.start], [input.start]);
  }

  const open: ScoredCell[] = [{ ...input.start, priority: 0, cost: 0 }];
  const cameFrom = new Map<string, string>();
  const costSoFar = new Map<string, number>([[cellKey(input.start), 0]]);
  const visited = new Set<string>();
  const visitedOrder: Cell[] = [];

  while (open.length > 0) {
    open.sort((a, b) => a.cost - b.cost || a.y - b.y || a.x - b.x);
    const current = open.shift()!;
    const currentKey = cellKey(current);
    if (visited.has(currentKey)) {
      continue;
    }
    visited.add(currentKey);
    visitedOrder.push({ x: current.x, y: current.y });

    if (sameCell(current, input.goal)) {
      return success(reconstructPath(cameFrom, currentKey), visitedOrder);
    }

    for (const neighbor of neighbors4(input, current)) {
      const newCost = current.cost + 1;
      const neighborKey = cellKey(neighbor);
      if (newCost >= (costSoFar.get(neighborKey) ?? Number.POSITIVE_INFINITY)) {
        continue;
      }
      cameFrom.set(neighborKey, currentKey);
      costSoFar.set(neighborKey, newCost);
      open.push({ ...neighbor, priority: newCost, cost: newCost });
    }
  }

  return failure(visitedOrder, "No path found");
}

function rrt(input: PlannerInput): PlannerResult {
  if (sameCell(input.start, input.goal)) {
    return success([input.start], [input.start]);
  }

  const random = seededRandom(12);
  const maxIterations = 1100;
  const goalSampleRate = 0.22;
  const parents = new Map<string, string>();
  const nodeKeys = new Set<string>([cellKey(input.start)]);
  const nodes: Cell[] = [input.start];
  const visitedOrder: Cell[] = [input.start];

  for (let i = 0; i < maxIterations; i += 1) {
    const sample = random() < goalSampleRate ? input.goal : randomFreeCell(input, random);
    const nearest = nodes.reduce((best, node) =>
      squaredDistance(node, sample) < squaredDistance(best, sample) ? node : best,
    );
    const candidate = steerOneCell(input, nearest, sample, nodeKeys);
    if (!candidate) {
      continue;
    }

    const candidateKey = cellKey(candidate);
    parents.set(candidateKey, cellKey(nearest));
    nodeKeys.add(candidateKey);
    nodes.push(candidate);
    visitedOrder.push(candidate);

    if (sameCell(candidate, input.goal)) {
      return success(reconstructPath(parents, candidateKey), visitedOrder);
    }
  }

  return failure(visitedOrder, "RRT did not connect start to goal within the iteration budget");
}

function neighbors4(input: PlannerInput, cell: Cell): Cell[] {
  return directions
    .map((direction) => ({ x: cell.x + direction.x, y: cell.y + direction.y }))
    .filter((candidate) => isTraversable(input, candidate));
}

function steerOneCell(
  input: PlannerInput,
  nearest: Cell,
  sample: Cell,
  existing: Set<string>,
): Cell | null {
  const candidates = neighbors4(input, nearest).filter((candidate) => !existing.has(cellKey(candidate)));
  candidates.sort((a, b) => squaredDistance(a, sample) - squaredDistance(b, sample) || a.y - b.y || a.x - b.x);
  return candidates[0] ?? null;
}

function randomFreeCell(input: PlannerInput, random: () => number): Cell {
  for (let attempts = 0; attempts < 200; attempts += 1) {
    const cell = {
      x: Math.floor(random() * input.width),
      y: Math.floor(random() * input.height),
    };
    if (isTraversable(input, cell)) {
      return cell;
    }
  }
  return input.start;
}

function isTraversable(input: PlannerInput, cell: Cell): boolean {
  return (
    cell.x >= 0 &&
    cell.x < input.width &&
    cell.y >= 0 &&
    cell.y < input.height &&
    !input.obstacles.has(cellKey(cell))
  );
}

function reconstructPath(cameFrom: Map<string, string>, currentKey: string): Cell[] {
  const pathKeys = [currentKey];
  while (cameFrom.has(currentKey)) {
    currentKey = cameFrom.get(currentKey)!;
    pathKeys.push(currentKey);
  }
  return pathKeys.reverse().map(parseCellKey);
}

function parseCellKey(key: string): Cell {
  const [x, y] = key.split(",").map(Number);
  return { x, y };
}

function success(path: Cell[], visited: Cell[]): PlannerResult {
  return {
    path,
    visited,
    cost: Math.max(0, path.length - 1),
    success: true,
    message: "Path found",
  };
}

function failure(visited: Cell[], message: string): PlannerResult {
  return {
    path: [],
    visited,
    cost: null,
    success: false,
    message,
  };
}

function manhattan(a: Cell, b: Cell): number {
  return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
}

function squaredDistance(a: Cell, b: Cell): number {
  return (a.x - b.x) ** 2 + (a.y - b.y) ** 2;
}

function seededRandom(seed: number): () => number {
  let state = seed >>> 0;
  return () => {
    state = (1664525 * state + 1013904223) >>> 0;
    return state / 0x100000000;
  };
}

