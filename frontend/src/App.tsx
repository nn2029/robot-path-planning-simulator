import { useEffect, useMemo, useRef, useState } from "react";
import type { PointerEvent } from "react";
import { cellKey, runPlanner, sameCell } from "./planners";
import type { Algorithm, Cell, InteractionMode, PlannerResult } from "./types";

const GRID_WIDTH = 30;
const GRID_HEIGHT = 20;
const CELL_SIZE = 24;
const DEFAULT_START: Cell = { x: 3, y: 9 };
const DEFAULT_GOAL: Cell = { x: 26, y: 9 };

const algorithmLabels: Record<Algorithm, string> = {
  astar: "A*",
  dijkstra: "Dijkstra",
  rrt: "RRT-style",
};

const modeLabels: Record<InteractionMode, string> = {
  obstacle: "Obstacle",
  erase: "Erase",
  start: "Start",
  goal: "Goal",
};

type RunStats = PlannerResult & {
  algorithm: Algorithm;
  durationMs: number;
};

export default function App() {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const [obstacles, setObstacles] = useState<Set<string>>(() => new Set());
  const [start, setStart] = useState<Cell>(DEFAULT_START);
  const [goal, setGoal] = useState<Cell>(DEFAULT_GOAL);
  const [mode, setMode] = useState<InteractionMode>("obstacle");
  const [algorithm, setAlgorithm] = useState<Algorithm>("astar");
  const [stats, setStats] = useState<RunStats | null>(null);
  const [isPainting, setIsPainting] = useState(false);

  const plannerInput = useMemo(
    () => ({
      width: GRID_WIDTH,
      height: GRID_HEIGHT,
      obstacles,
      start,
      goal,
    }),
    [obstacles, start, goal],
  );

  useEffect(() => {
    drawWorld(canvasRef.current, {
      obstacles,
      start,
      goal,
      path: stats?.path ?? [],
      visited: stats?.visited ?? [],
    });
  }, [goal, obstacles, start, stats]);

  function runSelectedPlanner() {
    const startedAt = performance.now();
    const result = runPlanner(algorithm, plannerInput);
    setStats({
      ...result,
      algorithm,
      durationMs: performance.now() - startedAt,
    });
  }

  function clearPath() {
    setStats(null);
  }

  function resetMap() {
    setObstacles(new Set());
    setStart(DEFAULT_START);
    setGoal(DEFAULT_GOAL);
    setStats(null);
  }

  function loadMaze() {
    const next = new Set<string>();

    for (let y = 2; y < GRID_HEIGHT - 2; y += 1) {
      if (y !== 14) {
        next.add(cellKey({ x: 10, y }));
      }
    }

    for (let y = 3; y < GRID_HEIGHT - 3; y += 1) {
      if (y !== 5) {
        next.add(cellKey({ x: 19, y }));
      }
    }

    for (let x = 13; x < 18; x += 1) {
      next.add(cellKey({ x, y: 8 }));
    }

    next.delete(cellKey(DEFAULT_START));
    next.delete(cellKey(DEFAULT_GOAL));
    setObstacles(next);
    setStart(DEFAULT_START);
    setGoal(DEFAULT_GOAL);
    setStats(null);
  }

  function clearObstacles() {
    setObstacles(new Set());
    setStats(null);
  }

  function handleCanvasPointer(event: PointerEvent<HTMLCanvasElement>) {
    const cell = eventToCell(event);
    if (!cell) {
      return;
    }
    applyCell(cell);
  }

  function applyCell(cell: Cell) {
    if (mode === "start") {
      if (!sameCell(cell, goal) && !obstacles.has(cellKey(cell))) {
        setStart(cell);
        setStats(null);
      }
      return;
    }

    if (mode === "goal") {
      if (!sameCell(cell, start) && !obstacles.has(cellKey(cell))) {
        setGoal(cell);
        setStats(null);
      }
      return;
    }

    setObstacles((current) => {
      const next = new Set(current);
      const key = cellKey(cell);
      if (mode === "erase") {
        next.delete(key);
      } else if (!sameCell(cell, start) && !sameCell(cell, goal)) {
        next.add(key);
      }
      return next;
    });
    setStats(null);
  }

  function eventToCell(event: PointerEvent<HTMLCanvasElement>): Cell | null {
    const rect = event.currentTarget.getBoundingClientRect();
    const x = Math.floor(((event.clientX - rect.left) / rect.width) * GRID_WIDTH);
    const y = Math.floor(((event.clientY - rect.top) / rect.height) * GRID_HEIGHT);
    if (x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT) {
      return null;
    }
    return { x, y };
  }

  return (
    <main className="app-shell">
      <section className="workspace">
        <header className="topbar">
          <div>
            <p className="eyebrow">Robotics Portfolio Project</p>
            <h1>2D Robot Path Planning Simulator</h1>
          </div>
          <div className="run-controls">
            <button className="primary" type="button" onClick={runSelectedPlanner}>
              Run
            </button>
            <button type="button" onClick={clearPath}>
              Clear path
            </button>
            <button type="button" onClick={loadMaze}>
              Maze
            </button>
            <button type="button" onClick={resetMap}>
              Reset
            </button>
          </div>
        </header>

        <div className="simulator-layout">
          <aside className="control-panel" aria-label="Simulator controls">
            <div className="control-group">
              <span className="control-label">Mode</span>
              <div className="segmented">
                {(Object.keys(modeLabels) as InteractionMode[]).map((item) => (
                  <button
                    className={mode === item ? "active" : ""}
                    key={item}
                    type="button"
                    onClick={() => setMode(item)}
                  >
                    <span className={`swatch ${item}`} />
                    {modeLabels[item]}
                  </button>
                ))}
              </div>
            </div>

            <div className="control-group">
              <label className="control-label" htmlFor="algorithm">
                Algorithm
              </label>
              <select
                id="algorithm"
                value={algorithm}
                onChange={(event) => setAlgorithm(event.target.value as Algorithm)}
              >
                {(Object.keys(algorithmLabels) as Algorithm[]).map((item) => (
                  <option key={item} value={item}>
                    {algorithmLabels[item]}
                  </option>
                ))}
              </select>
            </div>

            <div className="control-group">
              <span className="control-label">Map</span>
              <div className="button-stack">
                <button type="button" onClick={clearObstacles}>
                  Clear obstacles
                </button>
                <button type="button" onClick={loadMaze}>
                  Load maze
                </button>
              </div>
            </div>

            <StatsPanel stats={stats} obstacleCount={obstacles.size} />
          </aside>

          <section className="canvas-panel" aria-label="Planning grid">
            <canvas
              ref={canvasRef}
              width={GRID_WIDTH * CELL_SIZE}
              height={GRID_HEIGHT * CELL_SIZE}
              onPointerDown={(event) => {
                event.currentTarget.setPointerCapture(event.pointerId);
                setIsPainting(true);
                handleCanvasPointer(event);
              }}
              onPointerMove={(event) => {
                if (isPainting) {
                  handleCanvasPointer(event);
                }
              }}
              onPointerUp={(event) => {
                event.currentTarget.releasePointerCapture(event.pointerId);
                setIsPainting(false);
              }}
              onPointerCancel={() => setIsPainting(false)}
            />
          </section>
        </div>
      </section>
    </main>
  );
}

function StatsPanel({ stats, obstacleCount }: { stats: RunStats | null; obstacleCount: number }) {
  return (
    <div className="stats-panel">
      <span className="control-label">Stats</span>
      <dl>
        <div>
          <dt>Algorithm</dt>
          <dd>{stats ? algorithmLabels[stats.algorithm] : "Not run"}</dd>
        </div>
        <div>
          <dt>Status</dt>
          <dd className={stats?.success ? "success-text" : stats ? "warning-text" : ""}>
            {stats ? stats.message : "Ready"}
          </dd>
        </div>
        <div>
          <dt>Path cost</dt>
          <dd>{stats?.cost ?? "-"}</dd>
        </div>
        <div>
          <dt>Visited cells</dt>
          <dd>{stats?.visited.length ?? 0}</dd>
        </div>
        <div>
          <dt>Obstacles</dt>
          <dd>{obstacleCount}</dd>
        </div>
        <div>
          <dt>Runtime</dt>
          <dd>{stats ? `${stats.durationMs.toFixed(2)} ms` : "-"}</dd>
        </div>
      </dl>
    </div>
  );
}

function drawWorld(
  canvas: HTMLCanvasElement | null,
  world: {
    obstacles: Set<string>;
    start: Cell;
    goal: Cell;
    path: Cell[];
    visited: Cell[];
  },
) {
  if (!canvas) {
    return;
  }

  const ctx = canvas.getContext("2d");
  if (!ctx) {
    return;
  }

  const width = GRID_WIDTH * CELL_SIZE;
  const height = GRID_HEIGHT * CELL_SIZE;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#f8fafc";
  ctx.fillRect(0, 0, width, height);

  for (const cell of world.visited) {
    drawCell(ctx, cell, "rgba(37, 99, 235, 0.18)");
  }

  for (const key of world.obstacles) {
    const [x, y] = key.split(",").map(Number);
    drawCell(ctx, { x, y }, "#1f2937");
  }

  for (const cell of world.path) {
    drawCell(ctx, cell, "rgba(245, 158, 11, 0.62)");
  }
  drawPathLine(ctx, world.path);

  drawGrid(ctx);
  drawEndpoint(ctx, world.start, "#059669", "S");
  drawEndpoint(ctx, world.goal, "#dc2626", "G");
}

function drawGrid(ctx: CanvasRenderingContext2D) {
  ctx.strokeStyle = "#cbd5e1";
  ctx.lineWidth = 1;
  for (let x = 0; x <= GRID_WIDTH; x += 1) {
    const px = x * CELL_SIZE + 0.5;
    ctx.beginPath();
    ctx.moveTo(px, 0);
    ctx.lineTo(px, GRID_HEIGHT * CELL_SIZE);
    ctx.stroke();
  }
  for (let y = 0; y <= GRID_HEIGHT; y += 1) {
    const py = y * CELL_SIZE + 0.5;
    ctx.beginPath();
    ctx.moveTo(0, py);
    ctx.lineTo(GRID_WIDTH * CELL_SIZE, py);
    ctx.stroke();
  }
}

function drawCell(ctx: CanvasRenderingContext2D, cell: Cell, color: string) {
  ctx.fillStyle = color;
  ctx.fillRect(cell.x * CELL_SIZE + 1, cell.y * CELL_SIZE + 1, CELL_SIZE - 2, CELL_SIZE - 2);
}

function drawEndpoint(ctx: CanvasRenderingContext2D, cell: Cell, color: string, label: string) {
  const centerX = cell.x * CELL_SIZE + CELL_SIZE / 2;
  const centerY = cell.y * CELL_SIZE + CELL_SIZE / 2;
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(centerX, centerY, CELL_SIZE * 0.34, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = "#ffffff";
  ctx.font = "700 12px Inter, system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(label, centerX, centerY + 0.5);
}

function drawPathLine(ctx: CanvasRenderingContext2D, path: Cell[]) {
  if (path.length < 2) {
    return;
  }

  ctx.strokeStyle = "#b45309";
  ctx.lineWidth = 3;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.beginPath();
  path.forEach((cell, index) => {
    const x = cell.x * CELL_SIZE + CELL_SIZE / 2;
    const y = cell.y * CELL_SIZE + CELL_SIZE / 2;
    if (index === 0) {
      ctx.moveTo(x, y);
    } else {
      ctx.lineTo(x, y);
    }
  });
  ctx.stroke();
}
