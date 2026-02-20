# algorithms/pathfinding/bullseye_handler.py
#
# Handles the "bullseye" error case — image recognition detected bullseye,
# meaning the robot is at the wrong face of an obstacle.
#
# Two phases:
#   Phase 1 — Navigate from the robot's current position to the CORRECT face
#             of the bullseye obstacle and issue a SNAP there.
#             Uses self.grid (which contains ALL obstacles) for collision safety.
#   Phase 2 — From the resolved position, re-solve TSP and generate a fresh
#             path to visit all remaining unvisited obstacles.
#             Uses the SAME full-collision grid so A* never clips any obstacle.

from typing import List, Optional, Tuple

from algorithms.entities.grid import Grid
from algorithms.entities.obstacle import Obstacle
from algorithms.entities.robot import Robot
from algorithms.pathfinding.astar import AStar
from algorithms.commands.generator import CommandGenerator
from algorithms.utils.enums import Direction
from algorithms.utils.types import CellState


class BullseyeHandler:
    """
    Two-phase bullseye recovery orchestrator.

    self.grid is the FULL collision grid — it is built by main.py from every
    obstacle that was passed in remaining_obstacles (which always includes the
    bullseye obstacle itself).  It is NEVER stripped down; every A* call in
    every phase checks clearance against all obstacles on this grid.
    """

    def __init__(self, grid: Grid):
        self.grid    = grid           # full collision grid, never modified
        self.astar   = AStar(grid)    # bound to full grid permanently
        self.cmd_gen = CommandGenerator()

    # =========================================================================
    # PHASE 1 — PATH TO CORRECT FACE
    # =========================================================================

    def path_to_correct_face(
        self,
        obstacle: Obstacle,
        robot_current_state: CellState,
    ) -> Tuple[Optional[CellState], List[CellState], List[str]]:
        """
        Plan the shortest path from the robot's current position to the correct
        face of `obstacle` (obstacle.direction already set to the correct face).

        self.grid contains ALL obstacles, so is_reachable() and the full turn-
        sweep check in A* will reject any position that clips any obstacle,
        including the bullseye obstacle itself.

        Returns:
            resolved_state — CellState where robot ends up (None if unreachable)
            path_segment   — List[CellState] waypoints
            commands       — STM commands (SNAP appended, no FIN)
        """
        for retrying in [False, True]:
            candidates = obstacle.get_valid_viewing_positions(
                self.grid, retrying=retrying, specific_face=True
            )
            for candidate in candidates:
                path = self.astar.search(robot_current_state, candidate)
                if path:
                    cmds = self.cmd_gen.generate_commands(path)
                    cmds = [c for c in cmds if c != "FIN"]
                    cmds.append(f"SP{obstacle.obstacle_id}")
                    return candidate, path, cmds

        print(f"  [BullseyeHandler] Cannot reach correct face of obstacle {obstacle.obstacle_id}.")
        return None, [], [f"SNAP_FAILED{obstacle.obstacle_id}"]

    # =========================================================================
    # PHASE 2 — REROUTE REMAINING OBSTACLES
    # =========================================================================

    def reroute_remaining(
        self,
        robot_x: int,
        robot_y: int,
        robot_dir: int,
        visit_obstacles: List[Obstacle],
    ) -> dict:
        """
        Re-solve TSP and generate a complete path from the robot's current
        position to photograph every obstacle in `visit_obstacles`.

        KEY DESIGN:
          - self.grid is the FULL collision grid (all physical obstacles).
            HamiltonianSolver is initialised with self.grid so every A* call
            inside it checks clearance against every obstacle.
          - visit_obstacles is only the subset of obstacles that still need
            a SNAP.  We pass it as `target_obstacles` to the solver so it only
            generates viewing positions for those, but uses self.grid for all
            collision checks.

        Returns: { commands, path, distance }
        """
        from algorithms.pathfinding.hamiltonian import HamiltonianSolver

        if not visit_obstacles:
            return {"commands": ["FIN"], "path": [], "distance": 0.0}

        start_dir  = Direction(robot_dir) if robot_dir in [0, 2, 4, 6] else Direction.NORTH
        new_robot  = Robot(robot_x, robot_y, start_dir)

        # HamiltonianSolver gets self.grid (full collision grid).
        # target_obstacles tells it which obstacles to plan SNAP visits for.
        solver = HamiltonianSolver(self.grid, new_robot)
        permutation, total_cost = solver.find_optimal_order(
            target_obstacles=visit_obstacles
        )
        full_path = solver.generate_full_path(
            permutation,
            target_obstacles=visit_obstacles
        )
        raw_commands = self.cmd_gen.generate_commands(full_path)

        return {
            "commands": raw_commands,
            "path": [
                {"x": s.x, "y": s.y, "d": int(s.direction), "s": s.screenshot_id}
                for s in full_path
            ],
            "distance": total_cost,
        }

    # =========================================================================
    # COMBINED ENTRY POINT
    # =========================================================================

    def handle(
        self,
        obstacle_id: int,
        new_direction: int,
        robot_x: int,
        robot_y: int,
        robot_dir: int,
        remaining_obstacles_data: List[dict],
    ) -> dict:
        """
        Full bullseye recovery.

        self.grid was built by main.py from remaining_obstacles_data (which
        includes the bullseye obstacle), so it has every unvisited physical
        obstacle in it for collision checking.

        Args:
            obstacle_id              — ID of obstacle that showed bullseye
            new_direction            — TRUE correct image direction (0/2/4/6)
            robot_x/y/dir            — Robot's current physical position
            remaining_obstacles_data — All obstacles not yet fully visited,
                                       INCLUDING the bullseye obstacle

        Returns dict:
            full_path, full_commands  — stitched phase1+phase2 (use for playback)
            phase1_path, phase1_commands
            phase2_path, phase2_commands, phase2_distance
            resolved_position         — robot position after phase 1
            new_direction             — echo of the correct direction
            skipped_obstacle          — True if phase 1 was unreachable
        """
        start_dir   = Direction(robot_dir) if robot_dir in [0, 2, 4, 6] else Direction.NORTH
        robot_state = CellState(robot_x, robot_y, start_dir)

        # ---- Locate bullseye obstacle in self.grid and update its direction ----
        bullseye_obs = next(
            (o for o in self.grid.obstacles if o.obstacle_id == obstacle_id), None
        )
        if bullseye_obs is None:
            raise ValueError(f"Obstacle ID {obstacle_id} not found in grid.")

        bullseye_obs.direction = Direction(new_direction)
        print(f"[BullseyeHandler] Obs {obstacle_id}: direction → {bullseye_obs.direction.name}")

        # ---- Phase 1: navigate to correct face ----
        # self.astar is bound to self.grid (full collision grid), so this A*
        # search already respects every physical obstacle on the arena.
        resolved_state, p1_path_seg, p1_cmds = self.path_to_correct_face(
            bullseye_obs, robot_state
        )

        skipped = (resolved_state is None)
        if skipped:
            resolved_state = robot_state
            p1_path_seg    = [robot_state]

        print(
            f"[BullseyeHandler] Phase 1 done. "
            f"Resolved at ({resolved_state.x}, {resolved_state.y}, "
            f"{resolved_state.direction.name})"
        )

        # ---- Phase 2: build visit list and reroute ----
        # visit_obstacles: obstacles still needing a SNAP (bullseye obs excluded
        #                  because it was just handled in phase 1).
        # We fetch the live Obstacle objects from self.grid so directions are
        # already synced (bullseye obs direction was updated above).
        visit_obstacles: List[Obstacle] = []
        for obs_data in remaining_obstacles_data:
            if obs_data["id"] == obstacle_id:
                continue    # bullseye obs is now done
            grid_obs = next(
                (o for o in self.grid.obstacles if o.obstacle_id == obs_data["id"]),
                None,
            )
            if grid_obs is not None:
                visit_obstacles.append(grid_obs)
            # If for some reason the obstacle isn't in self.grid (shouldn't happen),
            # we skip it rather than silently using stale data.

        print(
            f"[BullseyeHandler] Phase 2: visiting {len(visit_obstacles)} obstacles, "
            f"collision-checking against {len(self.grid.obstacles)} total obstacles."
        )

        p2_result = self.reroute_remaining(
            resolved_state.x,
            resolved_state.y,
            int(resolved_state.direction),
            visit_obstacles=visit_obstacles,
        )

        # ---- Stitch phase1 + phase2 into one unified path ----
        p1_dicts = [
            {"x": s.x, "y": s.y, "d": int(s.direction), "s": s.screenshot_id}
            for s in p1_path_seg
        ]
        p2_path = p2_result["path"]

        # Avoid duplicating the join-point waypoint
        if p1_dicts and p2_path:
            combined_path = p1_dicts + p2_path[1:]
        else:
            combined_path = p1_dicts + p2_path

        combined_commands = p1_cmds + p2_result["commands"]

        return {
            "full_path":         combined_path,
            "full_commands":     combined_commands,
            "phase1_path":       p1_dicts,
            "phase1_commands":   p1_cmds,
            "phase2_path":       p2_path,
            "phase2_commands":   p2_result["commands"],
            "phase2_distance":   p2_result["distance"],
            "resolved_position": resolved_state.get_dict(),
            "new_direction":     new_direction,
            "skipped_obstacle":  skipped,
        }
