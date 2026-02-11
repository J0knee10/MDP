# main.py
import uvicorn
from typing import List, Optional
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# Internal imports (Adjust paths if your folder structure differs)
from algorithms.commands.generator import CommandGenerator
from algorithms.entities.grid import Grid
from algorithms.entities.obstacle import Obstacle
from algorithms.entities.robot import Robot
from algorithms.pathfinding.hamiltonian import HamiltonianSolver
from algorithms.utils.enums import Direction

app = FastAPI(title="MDP Algorithm Server")

# ALLOW CONNECTION FROM SIMULATOR AND ANDROID
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allows all origins (Sim on 3000, Android, etc.)
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- PYDANTIC MODELS (INPUT VALIDATION) ---

class ObstacleInput(BaseModel):
    id: int
    x: int
    y: int
    d: int

class AlgorithmInput(BaseModel):
    # This matches the JSON sent by Android/Simulator
    obstacles: List[ObstacleInput]
    robot_x: Optional[int] = 1   # Default to 1 if missing
    robot_y: Optional[int] = 1   # Default to 1 if missing
    robot_dir: Optional[int] = 0 # Default to 0 (North) if missing
    retrying: Optional[bool] = False

class PathPoint(BaseModel):
    x: int
    y: int
    d: int
    s: int

class AlgorithmOutput(BaseModel):
    commands: List[str]
    path: List[PathPoint]
    distance: float

# --- CORE ALGORITHM ORCHESTRATOR ---

def run_algorithm(obstacles_data: List[dict], robot_x: int, robot_y: int, robot_dir: int, retrying: bool) -> dict:
    """
    Orchestrates the pathfinding process:
    1. Setup Grid & Obstacles
    2. Setup Robot at the specific Android coordinates
    3. Solve TSP (Hamiltonian)
    4. Generate A* Paths
    5. Convert to Commands
    """
    # 1. Initialize Grid
    grid = Grid()
    
    # 2. Add Obstacles
    for obs in obstacles_data:
        grid.add_obstacle(Obstacle(
            obs["x"], 
            obs["y"], 
            Direction(obs["d"]), 
            obs["id"]
        ))
    
    # 3. Initialize Robot (FIX: Use input arguments, not hardcoded 1,1)
    # Ensure direction is converted to Enum
    start_dir = Direction(robot_dir) if robot_dir in [0, 2, 4, 6] else Direction.NORTH
    robot = Robot(robot_x, robot_y, start_dir)
    
    # 4. Solve for Path
    solver = HamiltonianSolver(grid, robot)
    
    # Note: find_optimal_order might need to handle the 'retrying' flag if your logic supports it
    permutation, total_cost = solver.find_optimal_order()
    full_path = solver.generate_full_path(permutation)
    
    # 5. Generate Commands
    cmd_gen = CommandGenerator()
    raw_commands = cmd_gen.generate_commands(full_path)
    # No need to call compress_commands separately if generate_commands calls it internally
    # But if your generator logic separated them, keep it. 
    # Based on my previous fix, generate_commands calls compress, so we are good.
    
    # 6. Format Output
    return {
        "commands": raw_commands,
        "path": [
            {
                "x": s.x, 
                "y": s.y, 
                "d": int(s.direction), 
                "s": s.screenshot_id
            } 
            for s in full_path
        ],
        "distance": total_cost
    }

# --- API ENDPOINTS ---

@app.get("/status")
def health_check():
    return {"status": "ok", "message": "Algorithm server is running (90-Degree Logic)"}

@app.post("/path", response_model=AlgorithmOutput)
def compute_path(input_data: AlgorithmInput):
    try:
        # Map Pydantic model to list of dicts
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
        import traceback
        traceback.print_exc() # Print full error to terminal
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    # Host 0.0.0.0 is crucial for RPi/Android connection
    uvicorn.run(app, host="0.0.0.0", port=5000)