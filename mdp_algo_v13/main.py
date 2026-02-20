# main.py
import uvicorn
from typing import List, Optional
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from algorithms.commands.generator import CommandGenerator
from algorithms.entities.grid import Grid
from algorithms.entities.obstacle import Obstacle
from algorithms.entities.robot import Robot
from algorithms.pathfinding.hamiltonian import HamiltonianSolver
from algorithms.pathfinding.bullseye_handler import BullseyeHandler
from algorithms.utils.enums import Direction

app = FastAPI(title="MDP Algorithm Server")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# =============================================================================
# PYDANTIC MODELS
# =============================================================================

class ObstacleInput(BaseModel):
    id: int
    x: int
    y: int
    d: int

class AlgorithmInput(BaseModel):
    obstacles: List[ObstacleInput]
    robot_x: Optional[int] = 1
    robot_y: Optional[int] = 1
    robot_dir: Optional[int] = 0
    retrying: Optional[bool] = False

class PathPoint(BaseModel):
    x: int
    y: int
    d: int
    s: int

class AlgorithmData(BaseModel):
    commands: List[str]
    snap_positions: List[PathPoint]

class AlgorithmOutput(BaseModel):
    data: AlgorithmData
    path: List[PathPoint]
    distance: float


class BullseyeInput(BaseModel):
    # Which obstacle triggered the bullseye
    obstacle_id: int

    # The TRUE correct image direction after randomisation (0/2/4/6)
    # The dashboard randomises this locally and sends it here so the server
    # can plan the path straight to the correct face.
    new_direction: int

    # Robot's CURRENT physical position when bullseye was detected
    robot_x: int
    robot_y: int
    robot_dir: int  # 0=NORTH, 2=EAST, 4=SOUTH, 6=WEST

    # ALL obstacles not yet successfully photographed, INCLUDING the bullseye
    # obstacle (with its updated direction already set).
    # Android/RPi tracks which obstacles remain unvisited.
    remaining_obstacles: List[ObstacleInput]


class BullseyeOutput(BaseModel):
    data: AlgorithmData
    # Unified path for playback (phase1 + phase2 stitched together)
    full_path: List[PathPoint]
    full_commands: List[str]

    # Individual phases (for debugging / display)
    phase1_path: List[PathPoint]
    phase1_commands: List[str]
    phase2_path: List[PathPoint]
    phase2_commands: List[str]
    phase2_distance: float

    resolved_position: PathPoint
    new_direction: int
    skipped_obstacle: bool


# =============================================================================
# CORE ALGORITHM (unchanged)
# =============================================================================

def run_algorithm(
    obstacles_data: List[dict],
    robot_x: int,
    robot_y: int,
    robot_dir: int,
    retrying: bool,
) -> dict:
    grid = Grid()
    for obs in obstacles_data:
        grid.add_obstacle(Obstacle(
            obs["x"], obs["y"], Direction(obs["d"]), obs["id"]
        ))

    start_dir = Direction(robot_dir) if robot_dir in [0, 2, 4, 6] else Direction.NORTH
    robot = Robot(robot_x, robot_y, start_dir)

    solver = HamiltonianSolver(grid, robot)
    permutation, total_cost = solver.find_optimal_order()
    full_path = solver.generate_full_path(permutation)

    cmd_gen = CommandGenerator()
    raw_commands = cmd_gen.generate_commands(full_path)

    path_points = [
        {"x": s.x, "y": s.y, "d": int(s.direction), "s": s.screenshot_id}
        for s in full_path
    ]

    return {
        "data": {
            "commands": raw_commands,
            "snap_positions": [p for p in path_points if p["s"] != -1]
        },
        "path": path_points,
        "distance": total_cost
    }


# =============================================================================
# ENDPOINTS
# =============================================================================

@app.get("/status")
def health_check():
    return {"status": "ok", "message": "Algorithm server is running"}


@app.post("/path", response_model=AlgorithmOutput)
def compute_path(input_data: AlgorithmInput):
    try:
        obstacles_data = [
            {"id": o.id, "x": o.x, "y": o.y, "d": o.d}
            for o in input_data.obstacles
        ]
        result = run_algorithm(
            obstacles_data,
            input_data.robot_x,
            input_data.robot_y,
            input_data.robot_dir,
            input_data.retrying
        )
        return result
    except Exception as e:
        import traceback; traceback.print_exc()
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/bullseye", response_model=BullseyeOutput)
def handle_bullseye(input_data: BullseyeInput):
    """
    Called when image recognition detects a BULLSEYE on the current obstacle.

    The caller (dashboard or Android) has already:
      1. Picked a new random/correct direction for the obstacle
      2. Updated remaining_obstacles with that new direction

    This endpoint:
      - Phase 1: Plans path from robot's current position to the obstacle's
                 correct face and issues a SNAP command there.
      - Phase 2: Re-solves TSP from resolved position for all remaining obstacles.
      - Returns a unified path + commands covering both phases.

    The dashboard replaces its active interpolated_path with full_path so the
    robot animation seamlessly continues from where it stopped.
    """
    try:
        # Build grid from ALL remaining obstacles (including bullseye obs)
        # so collision checking is accurate during face-approach navigation.
        grid = Grid()
        remaining_data = [
            {"id": o.id, "x": o.x, "y": o.y, "d": o.d}
            for o in input_data.remaining_obstacles
        ]
        for obs in remaining_data:
            grid.add_obstacle(Obstacle(
                obs["x"], obs["y"], Direction(obs["d"]), obs["id"]
            ))

        handler = BullseyeHandler(grid)
        result = handler.handle(
            obstacle_id=input_data.obstacle_id,
            new_direction=input_data.new_direction,
            robot_x=input_data.robot_x,
            robot_y=input_data.robot_y,
            robot_dir=input_data.robot_dir,
            remaining_obstacles_data=remaining_data,
        )
        # Wrap result for RPi
        result["data"] = {
            "commands": result["full_commands"],
            "snap_positions": [p for p in result["full_path"] if p["s"] != -1]
        }
        return result

    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        import traceback; traceback.print_exc()
        raise HTTPException(status_code=500, detail=str(e))


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=5000)
