# algorithms/entities/obstacle.py
from typing import List
from algorithms.utils.consts import (
    CELL_SIZE, 
    OBSTACLE_SIZE,
    ROBOT_CAMERA_DISTANCE, 
    EXPANDED_CELL,
    SCREENSHOT_COST
)
from algorithms.utils.enums import Direction
from algorithms.utils.types import CellState

class Obstacle:
    def __init__(self, x: int, y: int, direction: Direction, obstacle_id: int):
        self.x = x
        self.y = y
        self.direction = direction
        self.obstacle_id = obstacle_id
    
    def get_viewing_positions(self, retrying: bool = False) -> List[CellState]:
        """
        Generates valid spots for the robot to stand and take a photo.
        """
        positions = []
        
        # 1. Calculate the required distance (Center-to-Center)
        # We assume the robot's camera is roughly at the robot's front edge.
        # Distance = (Half Obstacle) + (Camera Focus Distance) + (Half Robot/Safety)
        # We roughly convert this to grid cells.
        
        # Note: We do NOT add EXPANDED_CELL here anymore. 
        # That was pushing the robot too far back.
        if not retrying:
            # Optimal: ~25-30cm from face
            dist_cells = (ROBOT_CAMERA_DISTANCE + OBSTACLE_SIZE // 2) // CELL_SIZE
            # Ensure at least 3 cells (30cm) for 90-degree turn clearance if needed
            if dist_cells < 3: dist_cells = 3
        else:
            # Retry: Step back 10cm
            dist_cells = ((ROBOT_CAMERA_DISTANCE + OBSTACLE_SIZE // 2) // CELL_SIZE) + 1

        offset_1 = dist_cells
        offset_2 = dist_cells + 1 # Alternative spot slightly further back

        # 2. Generate Coordinates based on Direction
        
        # If Image faces NORTH, Robot must be NORTH of it? 
        # NO! If image is on North face, robot must stand NORTH of it, facing SOUTH.
        
        if self.direction == Direction.NORTH:
            # Target: Robot stands at (x, y + offset), faces SOUTH
            target_d = Direction.SOUTH
            
            # Primary (Center)
            positions.append(CellState(self.x, self.y + offset_1, target_d, self.obstacle_id, 0))
            positions.append(CellState(self.x, self.y + offset_2, target_d, self.obstacle_id, 5))
            
            # Angled (Left/Right)
            positions.append(CellState(self.x - 1, self.y + offset_1, target_d, self.obstacle_id, SCREENSHOT_COST))
            positions.append(CellState(self.x + 1, self.y + offset_1, target_d, self.obstacle_id, SCREENSHOT_COST))

        elif self.direction == Direction.SOUTH:
            # Target: Robot stands at (x, y - offset), faces NORTH
            target_d = Direction.NORTH
            
            positions.append(CellState(self.x, self.y - offset_1, target_d, self.obstacle_id, 0))
            positions.append(CellState(self.x, self.y - offset_2, target_d, self.obstacle_id, 5))
            
            positions.append(CellState(self.x - 1, self.y - offset_1, target_d, self.obstacle_id, SCREENSHOT_COST))
            positions.append(CellState(self.x + 1, self.y - offset_1, target_d, self.obstacle_id, SCREENSHOT_COST))

        elif self.direction == Direction.EAST:
            # Target: Robot stands at (x + offset, y), faces WEST
            target_d = Direction.WEST
            
            positions.append(CellState(self.x + offset_1, self.y, target_d, self.obstacle_id, 0))
            positions.append(CellState(self.x + offset_2, self.y, target_d, self.obstacle_id, 5))
            
            positions.append(CellState(self.x + offset_1, self.y - 1, target_d, self.obstacle_id, SCREENSHOT_COST))
            positions.append(CellState(self.x + offset_1, self.y + 1, target_d, self.obstacle_id, SCREENSHOT_COST))

        elif self.direction == Direction.WEST:
            # Target: Robot stands at (x - offset, y), faces EAST
            target_d = Direction.EAST
            
            positions.append(CellState(self.x - offset_1, self.y, target_d, self.obstacle_id, 0))
            positions.append(CellState(self.x - offset_2, self.y, target_d, self.obstacle_id, 5))
            
            positions.append(CellState(self.x - offset_1, self.y - 1, target_d, self.obstacle_id, SCREENSHOT_COST))
            positions.append(CellState(self.x - offset_1, self.y + 1, target_d, self.obstacle_id, SCREENSHOT_COST))

        return positions

    def get_valid_viewing_positions(self, grid) -> List[CellState]:
        # Filter out positions that are out of bounds or colliding
        candidates = self.get_viewing_positions()
        valid = [pos for pos in candidates if grid.is_reachable(pos.x, pos.y)]
        return valid