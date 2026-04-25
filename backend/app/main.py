"""FastAPI entrypoint for the robot path planning simulator."""

from __future__ import annotations

from math import isfinite
from typing import Literal

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

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


class PlanResponse(BaseModel):
    algorithm: AlgorithmName
    success: bool
    message: str
    cost: float | None
    path: list[Coordinate]
    visited: list[Coordinate]


app = FastAPI(
    title="2D Robot Path Planning Simulator API",
    version="0.1.0",
    description="Grid-based A*, Dijkstra, and RRT-style path planning endpoints.",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
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
    try:
        grid = GridMap(
            width=request.width,
            height=request.height,
            obstacles=frozenset(cell.as_tuple() for cell in request.obstacles),
        )
        start = request.start.as_tuple()
        goal = request.goal.as_tuple()

        if request.algorithm == "astar":
            result = astar(grid, start, goal)
        elif request.algorithm == "dijkstra":
            result = dijkstra(grid, start, goal)
        else:
            result = GridRRTPlanner().plan(grid, start, goal)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    return PlanResponse(
        algorithm=request.algorithm,
        success=result.success,
        message=result.message,
        cost=result.cost if isfinite(result.cost) else None,
        path=[Coordinate(x=x, y=y) for x, y in result.path],
        visited=[Coordinate(x=x, y=y) for x, y in result.visited],
    )

