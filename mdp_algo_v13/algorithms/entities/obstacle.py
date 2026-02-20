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

    # -------------------------------------------------------------------------
    # Helper used by BullseyeHandler
    # -------------------------------------------------------------------------

    def get_faces_except(self, exclude_direction: Direction) -> List[Direction]:
        """
        Return the 3 face directions excluding `exclude_direction`.
        Adjacent faces come before the directly opposite face (easier to reach).

        Used by BullseyeHandler to know which faces to search after a bullseye
        is detected on `exclude_direction`.
        """
        all_faces = [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST]
        remaining = [f for f in all_faces if f != exclude_direction]
        opposite = {
            Direction.NORTH: Direction.SOUTH,
            Direction.SOUTH: Direction.NORTH,
            Direction.EAST:  Direction.WEST,
            Direction.WEST:  Direction.EAST,
        }[exclude_direction]
        # Sort so the face directly opposite is tried last
        remaining.sort(key=lambda f: (1 if f == opposite else 0))
        return remaining

    # -------------------------------------------------------------------------
    # Viewing position generation (unchanged)
    # -------------------------------------------------------------------------

    def get_viewing_positions(self, retrying: bool = False, specific_face: bool = True) -> List[CellState]:
        """
        Generates candidate spots for the robot to stand and photograph this obstacle.

        specific_face=True  → only generate positions for self.direction (known image face)
        specific_face=False → generate positions for all 4 faces
        """
        positions = []

        if not retrying:
            dist_cells = (ROBOT_CAMERA_DISTANCE + OBSTACLE_SIZE // 2) // CELL_SIZE
            if dist_cells < 3:
                dist_cells = 3
        else:
            dist_cells = ((ROBOT_CAMERA_DISTANCE + OBSTACLE_SIZE // 2) // CELL_SIZE) + 1

        offset_1 = dist_cells
        offset_2 = dist_cells + 1

        faces_to_check = [self.direction] if specific_face else [
            Direction.NORTH, Direction.SOUTH, Direction.EAST, Direction.WEST
        ]

        for face in faces_to_check:
            if face == Direction.NORTH:
                target_d = Direction.SOUTH
                positions.append(CellState(self.x,     self.y + offset_1, target_d, self.obstacle_id, 0))
                positions.append(CellState(self.x,     self.y + offset_2, target_d, self.obstacle_id, 5))
                positions.append(CellState(self.x - 1, self.y + offset_1, target_d, self.obstacle_id, SCREENSHOT_COST))
                positions.append(CellState(self.x + 1, self.y + offset_1, target_d, self.obstacle_id, SCREENSHOT_COST))

            elif face == Direction.SOUTH:
                target_d = Direction.NORTH
                positions.append(CellState(self.x,     self.y - offset_1, target_d, self.obstacle_id, 0))
                positions.append(CellState(self.x,     self.y - offset_2, target_d, self.obstacle_id, 5))
                positions.append(CellState(self.x - 1, self.y - offset_1, target_d, self.obstacle_id, SCREENSHOT_COST))
                positions.append(CellState(self.x + 1, self.y - offset_1, target_d, self.obstacle_id, SCREENSHOT_COST))

            elif face == Direction.EAST:
                target_d = Direction.WEST
                positions.append(CellState(self.x + offset_1, self.y,     target_d, self.obstacle_id, 0))
                positions.append(CellState(self.x + offset_2, self.y,     target_d, self.obstacle_id, 5))
                positions.append(CellState(self.x + offset_1, self.y - 1, target_d, self.obstacle_id, SCREENSHOT_COST))
                positions.append(CellState(self.x + offset_1, self.y + 1, target_d, self.obstacle_id, SCREENSHOT_COST))

            elif face == Direction.WEST:
                target_d = Direction.EAST
                positions.append(CellState(self.x - offset_1, self.y,     target_d, self.obstacle_id, 0))
                positions.append(CellState(self.x - offset_2, self.y,     target_d, self.obstacle_id, 5))
                positions.append(CellState(self.x - offset_1, self.y - 1, target_d, self.obstacle_id, SCREENSHOT_COST))
                positions.append(CellState(self.x - offset_1, self.y + 1, target_d, self.obstacle_id, SCREENSHOT_COST))

        return positions

    def get_valid_viewing_positions(
        self, grid, retrying: bool = False, specific_face: bool = True
    ) -> List[CellState]:
        """Filter candidates to those within bounds and not colliding."""
        candidates = self.get_viewing_positions(retrying, specific_face)
        return [pos for pos in candidates if grid.is_reachable(pos.x, pos.y)]
