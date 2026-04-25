"""Runtime limits for the planning API."""

from __future__ import annotations

from dataclasses import dataclass
import os


def _parse_origins(raw_origins: str) -> list[str]:
    return [origin.strip() for origin in raw_origins.split(",") if origin.strip()]


def _as_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, default))
    except ValueError:
        return default


@dataclass(frozen=True)
class PlannerSettings:
    cors_origins: list[str]
    max_grid_cells: int = 10_000
    max_obstacles: int = 5_000
    max_expansions: int = 8_000
    rrt_max_iterations: int = 1_200

    @classmethod
    def from_env(cls) -> "PlannerSettings":
        return cls(
            cors_origins=_parse_origins(
                os.getenv(
                    "CORS_ORIGINS",
                    "http://localhost:5173,http://127.0.0.1:5173",
                )
            ),
            max_grid_cells=_as_int("MAX_GRID_CELLS", 10_000),
            max_obstacles=_as_int("MAX_OBSTACLES", 5_000),
            max_expansions=_as_int("MAX_EXPANSIONS", 8_000),
            rrt_max_iterations=_as_int("RRT_MAX_ITERATIONS", 1_200),
        )


def get_settings() -> PlannerSettings:
    return PlannerSettings.from_env()
