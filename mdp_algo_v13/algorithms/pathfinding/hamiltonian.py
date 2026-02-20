import itertools
import numpy as np
from python_tsp.exact import solve_tsp_dynamic_programming
from typing import List, Optional, Tuple
from algorithms.entities.grid import Grid
from algorithms.entities.robot import Robot
from algorithms.entities.obstacle import Obstacle
from algorithms.pathfinding.astar import AStar
from algorithms.utils.types import CellState
from algorithms.utils.enums import Direction


class HamiltonianSolver:
    """
    Solves the TSP ordering problem and generates the full A* path.

    The grid passed to __init__ is the COLLISION grid — it must contain every
    physical obstacle on the arena so that A* never plans through any of them.

    Optionally, a separate `target_obstacles` list can be supplied to
    find_optimal_order() and generate_full_path().  When provided, the solver
    only generates SNAP viewing positions for those obstacles (the ones still
    needing a photo), while still using the full collision grid for all
    is_reachable() / A* sweep checks.

    If `target_obstacles` is None the solver behaves exactly as before,
    using self.grid.obstacles for both collision and TSP targets.
    """

    def __init__(self, grid: Grid, robot: Robot):
        self.grid  = grid          # COLLISION grid — never modified, always full
        self.robot = robot
        self.astar = AStar(grid)   # A* is bound to the collision grid permanently

    # ------------------------------------------------------------------
    # Internal helper: resolve which obstacle list to iterate for SNAP targets
    # ------------------------------------------------------------------
    def _target_list(self, target_obstacles: Optional[List[Obstacle]]) -> List[Obstacle]:
        """Return target_obstacles if given, else fall back to full grid list."""
        return target_obstacles if target_obstacles is not None else self.grid.obstacles

    # ------------------------------------------------------------------

    def generate_cost_matrix(
        self,
        positions: List[CellState],
    ) -> np.ndarray:
        n = len(positions)
        cost_matrix = np.zeros((n, n))

        for i in range(n):
            for j in range(n):
                if i == j:
                    cost_matrix[i][j] = 0
                    continue
                if j == 0:
                    cost_matrix[i][j] = 0
                    continue

                if positions[i].x == -99 or positions[j].x == -99:
                    cost_matrix[i][j] = 1e9
                    continue

                path = self.astar.search(positions[i], positions[j])
                if path:
                    cost = self.astar.cost_cache[(positions[i], positions[j])]
                    if hasattr(positions[j], 'penalty'):
                        cost += positions[j].penalty
                    cost_matrix[i][j] = cost
                else:
                    cost_matrix[i][j] = 1e9

        return cost_matrix

    def find_optimal_order(
        self,
        retrying: bool = False,
        target_obstacles: Optional[List[Obstacle]] = None,
    ) -> Tuple[List[int], float]:
        """
        target_obstacles: if supplied, only these obstacles are used as SNAP
                          targets (viewing positions generated for them only).
                          All is_reachable() calls still use the full collision
                          grid (self.grid), so the robot avoids every obstacle.
        """
        targets = self._target_list(target_obstacles)

        start_state = self.robot.get_start_state()
        viewing_positions = [start_state]

        for obstacle in targets:
            candidates = obstacle.get_viewing_positions(retrying=retrying)
            # is_reachable uses self.grid — the FULL collision grid
            valid_positions = [
                pos for pos in candidates if self.grid.is_reachable(pos.x, pos.y)
            ]

            selected_pos = None
            if valid_positions:
                for pos in valid_positions:
                    if self.astar.search(start_state, pos):
                        selected_pos = pos
                        break
                if not selected_pos:
                    selected_pos = valid_positions[0]

            if not selected_pos:
                print(f"⚠️ Warning: Obstacle {obstacle.obstacle_id} has NO safe viewing spots!")
                viewing_positions.append(CellState(-99, -99, Direction.NORTH))
            else:
                viewing_positions.append(selected_pos)

        print("🧩 Solving for Fastest Path (Smart-Subset TSP)...")
        cost_matrix  = self.generate_cost_matrix(viewing_positions)
        n_targets    = len(targets)

        best_permutation = None
        best_distance    = float('inf')

        for subset_size in range(n_targets, 0, -1):
            found_valid_subset = False
            min_subset_dist    = float('inf')
            best_subset_perm   = None

            for comb in itertools.combinations(range(1, n_targets + 1), subset_size):
                subset_indices = [0] + list(comb)
                sub_matrix     = cost_matrix[np.ix_(subset_indices, subset_indices)]
                perm, dist     = solve_tsp_dynamic_programming(sub_matrix)

                if dist < 1e9:
                    found_valid_subset = True
                    if dist < min_subset_dist:
                        min_subset_dist  = dist
                        best_subset_perm = [subset_indices[i] for i in perm]

            if found_valid_subset:
                best_permutation = best_subset_perm
                best_distance    = min_subset_dist
                print(f"🚀 OPTIMAL PATH FOUND for {subset_size}/{n_targets} obstacles! Cost: {best_distance:.2f}")

                skipped = set(range(1, n_targets + 1)) - set(best_permutation)
                if skipped:
                    skipped_ids = [targets[i - 1].obstacle_id for i in skipped]
                    print(f"🛑 SKIPPED TRAPPED OBSTACLES: {skipped_ids}")
                break

        if best_permutation is None:
            print("💀 ALL PATHS FAILED. Robot is completely boxed in.")
            return [0], 0

        if best_permutation[0] != 0:
            start_idx        = best_permutation.index(0)
            best_permutation = best_permutation[start_idx:] + best_permutation[:start_idx]

        return best_permutation, best_distance

    def generate_full_path(
        self,
        permutation: List[int],
        target_obstacles: Optional[List[Obstacle]] = None,
    ) -> List[CellState]:
        """
        target_obstacles: same semantics as in find_optimal_order().
                          Must be the identical list used there so indices match.
        """
        targets = self._target_list(target_obstacles)

        start_state      = self.robot.get_start_state()
        viewing_positions = [start_state]

        for obstacle in targets:
            candidates     = obstacle.get_viewing_positions()
            valid_positions = [
                pos for pos in candidates if self.grid.is_reachable(pos.x, pos.y)
            ]

            selected_pos = None
            if valid_positions:
                for pos in valid_positions:
                    if self.astar.search(start_state, pos):
                        selected_pos = pos
                        break
                if not selected_pos:
                    selected_pos = valid_positions[0]

            viewing_positions.append(
                selected_pos if selected_pos else CellState(-99, -99, Direction.NORTH)
            )

        full_path = []
        for i in range(len(permutation) - 1):
            from_idx = permutation[i]
            to_idx   = permutation[i + 1]
            if to_idx == 0:
                continue

            segment = self.astar.search(
                viewing_positions[from_idx], viewing_positions[to_idx]
            )
            if not segment:
                continue

            if i == 0:
                full_path.extend(segment)
            else:
                full_path.extend(segment[1:])

            if to_idx > 0 and full_path:
                full_path[-1].screenshot_id = targets[to_idx - 1].obstacle_id

        return full_path
