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
    
    def get_viewing_positions(self, retrying: bool = False, specific_face: bool = True, ignored_faces: List[int] = None) -> List[CellState]:
        positions = []
        if ignored_faces is None:
            ignored_faces = []

        # --- BULLSEYE FILTER LOGIC ---
        if specific_face:
            faces_to_check = [self.direction]
        else:
            # Check all 4 sides, BUT filter out the ones we already know are bullseyes
            faces_to_check = [d for d in [Direction.NORTH, Direction.SOUTH, Direction.EAST, Direction.WEST] if d.value not in ignored_faces]

        for face in faces_to_check:
            if face == Direction.NORTH:
                target_d = Direction.SOUTH
                # 10cm gap (3 cells): Center ONLY
                positions.append(CellState(self.x, self.y + 3, target_d, self.obstacle_id, 0))
                # 20cm gap (4 cells): Center OR Off-center (+/- 1 grid)
                positions.append(CellState(self.x, self.y + 4, target_d, self.obstacle_id, 5))
                positions.append(CellState(self.x - 1, self.y + 4, target_d, self.obstacle_id, SCREENSHOT_COST))
                positions.append(CellState(self.x + 1, self.y + 4, target_d, self.obstacle_id, SCREENSHOT_COST))

            elif face == Direction.SOUTH:
                target_d = Direction.NORTH
                # 10cm gap (3 cells): Center ONLY
                positions.append(CellState(self.x, self.y - 3, target_d, self.obstacle_id, 0))
                # 20cm gap (4 cells): Center OR Off-center (+/- 1 grid)
                positions.append(CellState(self.x, self.y - 4, target_d, self.obstacle_id, 5))
                positions.append(CellState(self.x - 1, self.y - 4, target_d, self.obstacle_id, SCREENSHOT_COST))
                positions.append(CellState(self.x + 1, self.y - 4, target_d, self.obstacle_id, SCREENSHOT_COST))

            elif face == Direction.EAST:
                target_d = Direction.WEST
                # 10cm gap (3 cells): Center ONLY
                positions.append(CellState(self.x + 3, self.y, target_d, self.obstacle_id, 0))
                # 20cm gap (4 cells): Center OR Off-center (+/- 1 grid)
                positions.append(CellState(self.x + 4, self.y, target_d, self.obstacle_id, 5))
                positions.append(CellState(self.x + 4, self.y - 1, target_d, self.obstacle_id, SCREENSHOT_COST))
                positions.append(CellState(self.x + 4, self.y + 1, target_d, self.obstacle_id, SCREENSHOT_COST))

            elif face == Direction.WEST:
                target_d = Direction.EAST
                # 10cm gap (3 cells): Center ONLY
                positions.append(CellState(self.x - 3, self.y, target_d, self.obstacle_id, 0))
                # 20cm gap (4 cells): Center OR Off-center (+/- 1 grid)
                positions.append(CellState(self.x - 4, self.y, target_d, self.obstacle_id, 5))
                positions.append(CellState(self.x - 4, self.y - 1, target_d, self.obstacle_id, SCREENSHOT_COST))
                positions.append(CellState(self.x - 4, self.y + 1, target_d, self.obstacle_id, SCREENSHOT_COST))

        return positions

    def get_valid_viewing_positions(self, grid, retrying: bool = False, specific_face: bool = True, ignored_faces: List[int] = None) -> List[CellState]:
        candidates = self.get_viewing_positions(retrying, specific_face, ignored_faces)
        return [pos for pos in candidates if grid.is_reachable(pos.x, pos.y)]