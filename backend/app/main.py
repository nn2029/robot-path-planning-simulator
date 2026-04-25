"""FastAPI entrypoint for the robot path planning simulator."""

from __future__ import annotations

from math import isfinite
from time import perf_counter
from typing import Literal

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from app.config import get_settings
from app.planners import GridMap, GridRRTPlanner, astar, dijkstra

AlgorithmName = Literal["astar", "dijkstra", "rrt"]


class Coordinate(BaseModel):
    x: int
    y: int

    def as_tuple(self) -> tuple[int, int]:
        return (self.x, self.y)


class PlanRequest(BaseModel):
    width: int = Field(ge=1, le=200)
    height: int = Field(ge=1, le=200)
    start: Coordinate
    goal: Coordinate
    obstacles: list[Coordinate] = Field(default_factory=list)
    algorithm: AlgorithmName = "astar"
    max_expansions: int | None = Field(default=None, ge=1, le=100_000)


class PlanResponse(BaseModel):
    algorithm: AlgorithmName
    success: bool
    message: str
    cost: float | None
    path: list[Coordinate]
    visited: list[Coordinate]
    expanded_nodes: int
    duration_ms: float


settings = get_settings()
app = FastAPI(
    title="2D Robot Path Planning Simulator API",
    version="0.1.0",
    description="Grid-based A*, Dijkstra, and RRT-style path planning endpoints.",
)

app.add_middleware(
    CORSMiddleware,
    # The deployed frontend is set through CORS_ORIGINS; local dev keeps defaults.
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/health")
def health() -> dict[str, str]:
    return {"status": "ok"}


@app.get("/algorithms")
def algorithms() -> dict[str, list[str]]:
    return {"algorithms": ["astar", "dijkstra", "rrt"]}


@app.post("/plan", response_model=PlanResponse)
def plan_path(request: PlanRequest) -> PlanResponse:
    _validate_request_size(request)
    started_at = perf_counter()
    max_expansions = min(
        request.max_expansions or settings.max_expansions,
        settings.max_expansions,
    )

    try:
        grid = GridMap(
            width=request.width,
            height=request.height,
            obstacles=frozenset(cell.as_tuple() for cell in request.obstacles),
        )
        start = request.start.as_tuple()
        goal = request.goal.as_tuple()

        if request.algorithm == "astar":
            result = astar(grid, start, goal, max_expansions=max_expansions)
        elif request.algorithm == "dijkstra":
            result = dijkstra(grid, start, goal, max_expansions=max_expansions)
        else:
            result = GridRRTPlanner(
                max_iterations=min(settings.rrt_max_iterations, max_expansions)
            ).plan(grid, start, goal)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    return PlanResponse(
        algorithm=request.algorithm,
        success=result.success,
        message=result.message,
        cost=result.cost if isfinite(result.cost) else None,
        path=[Coordinate(x=x, y=y) for x, y in result.path],
        visited=[Coordinate(x=x, y=y) for x, y in result.visited],
        expanded_nodes=len(result.visited),
        duration_ms=round((perf_counter() - started_at) * 1000, 3),
    )


def _validate_request_size(request: PlanRequest) -> None:
    cells = request.width * request.height
    if cells > settings.max_grid_cells:
        raise HTTPException(
            status_code=413,
            detail=f"Grid has {cells} cells, above the {settings.max_grid_cells} cell limit",
        )
    if len(request.obstacles) > settings.max_obstacles:
        raise HTTPException(
            status_code=413,
            detail=(
                f"Request includes {len(request.obstacles)} obstacles, "
                f"above the {settings.max_obstacles} obstacle limit"
            ),
        )
