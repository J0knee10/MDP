# algorithms/pathfinding/hamiltonian.py

import numpy as np
from python_tsp.exact import solve_tsp_dynamic_programming
from typing import List, Tuple
from algorithms.entities.grid import Grid
from algorithms.entities.robot import Robot
from algorithms.pathfinding.astar import AStar
from algorithms.utils.types import CellState

class HamiltonianSolver:
    def __init__(self, grid: Grid, robot: Robot):
        self.grid = grid
        self.robot = robot
        self.astar = AStar(grid)
    
    def generate_cost_matrix(self, positions: List[CellState]) -> np.ndarray:
        n = len(positions)
        cost_matrix = np.zeros((n, n))
        
        # print("   [Calculating Distances...]") # Optional noise reduction
        for i in range(n):
            for j in range(n):
                if i == j:
                    cost_matrix[i][j] = 0
                    continue
                
                # SPEED TRICK: Cost to return to START (Node 0) is 0.
                # This tricks the solver into creating a one-way path.
                if j == 0:
                    cost_matrix[i][j] = 0
                    continue

                path = self.astar.search(positions[i], positions[j])
                if path:
                    cost = self.astar.cost_cache[(positions[i], positions[j])]
                    # Add penalty if the target view is "hard" (e.g. corner)
                    cost += positions[j].penalty
                    cost_matrix[i][j] = cost
                else:
                    cost_matrix[i][j] = 1e9 # Infinity
                    # print(f"   ⚠️ Unreachable: Node {i} -> Node {j}")

        return cost_matrix
    
    def find_optimal_order(self, retrying: bool = False) -> Tuple[List[int], float]:  # <--- Update signature
        start_state = self.robot.get_start_state()
        viewing_positions = [start_state]
        
        for obstacle in self.grid.obstacles:
            # Pass the retrying flag here!
            # We need to manually call get_viewing_positions with the flag, 
            # then filter for reachability.
            
            candidates = obstacle.get_viewing_positions(retrying=retrying) # <--- USE FLAG
            valid_positions = [pos for pos in candidates if self.grid.is_reachable(pos.x, pos.y)]
            
            if not valid_positions:
                print(f"❌ CRITICAL: Obstacle {obstacle.obstacle_id} unreachable.")
                viewing_positions.append(start_state)
            else:
                viewing_positions.append(valid_positions[0])
        
        # 2. Solve TSP
        print("🧩 Solving for Fastest Path...")
        cost_matrix = self.generate_cost_matrix(viewing_positions)
        permutation, distance = solve_tsp_dynamic_programming(cost_matrix)
        
        # 3. Check for Infinite Cost (Failure)
        if distance >= 1e9:
            print(f"💀 PATH FAILED. Total Cost: {distance} (Unreachable blocks detected)")
        else:
            print(f"🚀 OPTIMAL PATH FOUND! Estimated Cost: {distance}")
            path_str = " -> ".join([f"Start" if p==0 else f"Obs {self.grid.obstacles[p-1].obstacle_id}" for p in permutation])
            print(f"📍 Sequence: {path_str}")

        # 4. Rotate to ensure Start is first
        if permutation[0] != 0:
            start_idx = permutation.index(0)
            permutation = permutation[start_idx:] + permutation[:start_idx]
        
        return permutation, distance
    
    def generate_full_path(self, permutation: List[int]) -> List[CellState]:
        start_state = self.robot.get_start_state()
        viewing_positions = [start_state]
        
        for obstacle in self.grid.obstacles:
            positions = obstacle.get_valid_viewing_positions(self.grid)
            viewing_positions.append(positions[0] if positions else start_state)
        
        full_path = []
        
        # STOP AT LAST OBSTACLE (Do not add segment returning to 0)
        for i in range(len(permutation) - 1):
            from_idx = permutation[i]
            to_idx = permutation[i + 1]
            
            # If next stop is Start, IGNORE IT. We are done.
            if to_idx == 0:
                continue

            segment = self.astar.search(viewing_positions[from_idx], viewing_positions[to_idx])
            
            # Add to full path
            if i == 0:
                full_path.extend(segment)
            else:
                full_path.extend(segment[1:]) # Avoid duplicating the point
            
            # Mark the snapshot
            if to_idx > 0:
                full_path[-1].screenshot_id = self.grid.obstacles[to_idx - 1].obstacle_id
        
        return full_path