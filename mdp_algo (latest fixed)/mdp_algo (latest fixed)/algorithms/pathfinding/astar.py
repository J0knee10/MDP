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
        return self.f_cost < other.f_cost

class AStar:
    def __init__(self, grid: Grid):
        self.grid = grid
        # Cache to store the cost of calculated paths for the Hamiltonian solver
        self.cost_cache: Dict[Tuple[CellState, CellState], float] = {}

    def heuristic(self, current: CellState, goal: CellState) -> float:
        # Euclidean distance is best for 90-degree turns as it allows diagonal estimation
        return ((current.x - goal.x) ** 2 + (current.y - goal.y) ** 2) ** 0.5

    def get_neighbors(self, state: CellState) -> List[Tuple[CellState, float]]:
        neighbors = []
        x, y, d = state.x, state.y, state.direction
        r = TURN_RADIUS # 3 cells (30cm)

        # --- 1. STRAIGHT MOVEMENT (Forward & Backward) ---
        dx, dy = 0, 0
        if d == Direction.NORTH: dy = 1
        elif d == Direction.SOUTH: dy = -1
        elif d == Direction.EAST: dx = 1
        elif d == Direction.WEST: dx = -1
        
        # Check both Forward (sign=1) and Backward (sign=-1)
        for sign in [1, -1]:
            nx, ny = x + (dx * sign), y + (dy * sign)
            if self.grid.is_reachable(nx, ny):
                # Cost is 1 per step
                neighbors.append((CellState(nx, ny, d), 1))

        # --- 2. 90-DEGREE TURNS (Forward & Backward) ---
        # We define transitions for FL, FR, BL, BR
        
        def apply_turn(turn_type):
            # Returns (dx, dy, new_direction)
            if turn_type == 'FL': # Forward-Left
                moves = {0:(-r,r,6), 2:(r,r,0), 4:(r,-r,2), 6:(-r,-r,4)}
            elif turn_type == 'FR': # Forward-Right
                moves = {0:(r,r,2), 2:(r,-r,4), 4:(-r,-r,6), 6:(-r,r,0)}
            elif turn_type == 'BL': # Backward-Left (Reverse logic of FR)
                moves = {0:(r,-r,2), 2:(-r,-r,4), 4:(-r,r,6), 6:(r,r,0)}
            elif turn_type == 'BR': # Backward-Right (Reverse logic of FL)
                moves = {0:(-r,-r,6), 2:(-r,r,0), 4:(r,r,2), 6:(r,-r,4)}
            return moves[d]

        for turn in ['FL', 'FR', 'BL', 'BR']:
            tdx, tdy, new_d = apply_turn(turn)
            nx, ny = x + tdx, y + tdy
            
            # CHECK 1: Is the End Destination valid?
            if not self.grid.is_reachable(nx, ny, turn=True):
                continue

            # CHECK 2 (CRITICAL): Check the "Elbow" of the turn arc.
            # A 3x3 turn is roughly a diagonal move. We must ensure the 
            # space halfway through the turn (the corner) is also clear.
            # We approximate the midpoint by halving the displacement.
            mid_x = x + int(tdx / 2)
            mid_y = y + int(tdy / 2)

            if not self.grid.is_reachable(mid_x, mid_y, turn=True):
                continue
            
            # If safe, add the turn neighbor
            # Cost = TURN_COST + distance traveled (r)
            neighbors.append((CellState(nx, ny, Direction(new_d)), TURN_COST + r))

        return neighbors

    def search(self, start: CellState, goal: CellState) -> List[CellState]:
        open_set = []
        heapq.heappush(open_set, AStarNode(start, 0, self.heuristic(start, goal)))
        closed_set = set()
        
        # Track g_scores separately to avoid node object comparison issues
        g_scores = {start: 0}
        
        while open_set:
            current_node = heapq.heappop(open_set)
            curr = current_node.state
            
            # GOAL CHECK
            if curr.x == goal.x and curr.y == goal.y and curr.direction == goal.direction:
                # Save actual cost for Hamiltonian use
                self.cost_cache[(start, goal)] = current_node.g_cost
                return self._reconstruct_path(current_node)
            
            if curr in closed_set: continue
            closed_set.add(curr)
            
            for next_s, cost in self.get_neighbors(curr):
                if next_s in closed_set: continue
                
                tentative_g = g_scores[curr] + cost
                
                if next_s not in g_scores or tentative_g < g_scores[next_s]:
                    g_scores[next_s] = tentative_g
                    h_cost = self.heuristic(next_s, goal)
                    heapq.heappush(open_set, AStarNode(next_s, tentative_g, h_cost, current_node))
                    
        return [] # No path found

    def _reconstruct_path(self, node):
        path = []
        while node:
            path.append(node.state)
            node = node.parent
        return path[::-1]