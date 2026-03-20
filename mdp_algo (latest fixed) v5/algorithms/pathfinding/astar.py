import heapq
from typing import Dict, List, Optional, Tuple
from algorithms.entities.grid import Grid
from algorithms.utils.consts import TURN_RADIUS, TURN_COST
from algorithms.utils.enums import Direction
from algorithms.utils.types import CellState

class AStarNode:
    def __init__(self, state: CellState, g_cost: float, h_cost: float, parent: Optional['AStarNode'] = None):
        self.state = state
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost
        self.parent = parent
    
    def __lt__(self, other):
        # Strict Tie-Breaker to prevent randomized zig-zag paths
        if self.f_cost == other.f_cost:
            return self.h_cost < other.h_cost
        return self.f_cost < other.f_cost

class AStar:
    def __init__(self, grid: Grid):
        self.grid = grid
        self.cost_cache: Dict[Tuple[CellState, CellState], float] = {}

    def heuristic(self, current: CellState, goal: CellState) -> float:
        return ((current.x - goal.x) ** 2 + (current.y - goal.y) ** 2) ** 0.5

    def get_neighbors(self, state: CellState) -> List[Tuple[CellState, float]]:
        neighbors = []
        x, y, d = state.x, state.y, state.direction
        r = TURN_RADIUS # Typically 3 (30cm)

        # --- 1. STRAIGHT MOVEMENT ---
        dx, dy = 0, 0
        if d == Direction.NORTH: dy = 1
        elif d == Direction.SOUTH: dy = -1
        elif d == Direction.EAST: dx = 1
        elif d == Direction.WEST: dx = -1
        
        for sign in [1, -1]:
            nx, ny = x + (dx * sign), y + (dy * sign)
            if self.grid.is_reachable(nx, ny):
                neighbors.append((CellState(nx, ny, d), 1))

        # --- 2. 90-DEGREE TURNS ---
        def apply_turn(turn_type):
            if turn_type == 'FL':
                return {0:(-r,r,6), 2:(r,r,0), 4:(r,-r,2), 6:(-r,-r,4)}[d]
            elif turn_type == 'FR':
                return {0:(r,r,2), 2:(r,-r,4), 4:(-r,-r,6), 6:(-r,r,0)}[d]
            elif turn_type == 'BL':
                return {0:(-r,-r,2), 2:(-r,r,4), 4:(r,r,6), 6:(r,-r,0)}[d]
            elif turn_type == 'BR':
                return {0:(r,-r,6), 2:(-r,-r,0), 4:(-r,r,2), 6:(r,r,4)}[d]

        for turn in ['FL', 'FR', 'BL', 'BR']:
            tdx, tdy, new_d = apply_turn(turn)
            nx, ny = x + tdx, y + tdy
            
            # Check if final destination is safe
            if not self.grid.is_reachable(nx, ny, turn=True):
                continue

            # --- EXACT L-SHAPE HARDWARE MASK (The "6-Point" Rule) ---
            # Traces the precise center-point path to hit exactly what the hardware hits.
            # - Enforces STM pre-nudge rules (straight movement first).
            # - Sweeps deep into the corner to catch the (10cm, 50cm) nose swing.
            # - Skips the extreme outer corner (0cm, 50cm) to prevent wall clipping.
            # - Skips the 9 inner squares completely.
            safe = True
            step_x = 1 if tdx > 0 else -1
            step_y = 1 if tdy > 0 else -1
            
            check_points = []
            
            if d in [Direction.NORTH, Direction.SOUTH]:
                # Leg 1: The Pre-Nudge (Drives deep straight along Y)
                for i in range(1, r): 
                    check_points.append((0, i * step_y))
                # Leg 2: The Pivot Exit (Shifts to target X, drives along X)
                for i in range(1, r + 1):
                    check_points.append((i * step_x, r * step_y))
            else:
                # Leg 1: The Pre-Nudge (Drives deep straight along X)
                for i in range(1, r):
                    check_points.append((i * step_x, 0))
                # Leg 2: The Pivot Exit (Shifts to target Y, drives along Y)
                for i in range(1, r + 1):
                    check_points.append((r * step_x, i * step_y))
                    
            for cx, cy in check_points:
                if not self.grid.is_reachable(x + cx, y + cy, turn=True):
                    safe = False
                    break
            
            if not safe:
                continue
            
            # Slight penalty to reverse turns so A* prefers forward paths
            cost = TURN_COST + r + (2 if turn in ['BL', 'BR'] else 0)
            neighbors.append((CellState(nx, ny, Direction(new_d)), cost))

        return neighbors

    def search(self, start: CellState, goal: CellState) -> List[CellState]:
        open_set = []
        heapq.heappush(open_set, AStarNode(start, 0, self.heuristic(start, goal)))
        closed_set = set()
        g_scores = {start: 0}
        
        while open_set:
            current_node = heapq.heappop(open_set)
            curr = current_node.state
            
            if curr.x == goal.x and curr.y == goal.y and curr.direction == goal.direction:
                self.cost_cache[(start, goal)] = current_node.g_cost
                return self._reconstruct_path(current_node)
            
            if curr in closed_set: continue
            closed_set.add(curr)
            
            for next_s, cost in self.get_neighbors(curr):
                if next_s in closed_set: continue
                tentative_g = g_scores[curr] + cost
                if next_s not in g_scores or tentative_g < g_scores[next_s]:
                    g_scores[next_s] = tentative_g
                    heapq.heappush(open_set, AStarNode(next_s, tentative_g, self.heuristic(next_s, goal), current_node))
                    
        return []

    def _reconstruct_path(self, node):
        path = []
        while node:
            path.append(node.state)
            node = node.parent
        return path[::-1]