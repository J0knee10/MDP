import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.widgets as widgets
import matplotlib.transforms as transforms
import requests
import numpy as np

# CONFIGURATION
API_URL      = "http://localhost:5000/path"
BULLSEYE_URL = "http://localhost:5000/bullseye"
GRID_SIZE    = 20

# Direction int → human name
DIR_NAMES = {0: "NORTH", 2: "EAST", 4: "SOUTH", 6: "WEST"}
# Direction int → arrow drawing params  (ox, oy, adx, ady) all relative to cell origin
FACE_ARROW = {0: (0.5, 1, 0, 0.3), 2: (1, 0.5, 0.3, 0), 4: (0.5, 0, 0, -0.3), 6: (0, 0.5, -0.3, 0)}


class InteractiveDashboard:

    # =========================================================================
    # INIT
    # =========================================================================

    def __init__(self):
        # --- Obstacle & path state ---
        self.obstacles   = []          # List of {id, x, y, d}  — ground truth obstacle list
        self.raw_path    = []          # Active path being played back
        self.interpolated_path = []    # Smooth interpolated frames for the active path
        self.visited_ids = []          # Obstacle IDs in visit order (original plan)
        self.cost        = 0.0

        # --- Playback state ---
        self.current_frame = 0
        self.is_playing    = False

        # --- Bullseye state ---
        #   'normal'           — following original planned path
        #   'bullseye_recovery'— showing recovery path after bullseye trigger
        self.mode = 'normal'
        self.bullseye_obstacle_id  = None   # ID of the obstacle that triggered bullseye
        self.bullseye_new_dir      = None   # Randomised correct direction (int)
        self.phase1_path           = []     # Phase 1 path dicts (face approach)
        self.phase2_raw_path       = []     # Phase 2 path dicts (rerouted remaining)

        # ---- BUILD FIGURE ----
        self.fig, self.ax = plt.subplots(figsize=(10, 12))
        plt.subplots_adjust(bottom=0.26)

        self.timer = self.fig.canvas.new_timer(interval=50)
        self.timer.add_callback(self.play_step)

        # --- Row 1: Playback controls ---
        self.btn_prev = widgets.Button(plt.axes([0.05, 0.14, 0.12, 0.06]), '<< Prev')
        self.btn_prev.on_clicked(self.prev_step)

        self.btn_play = widgets.Button(plt.axes([0.19, 0.14, 0.12, 0.06]), 'Play', color='lightgreen')
        self.btn_play.on_clicked(self.toggle_play)

        self.btn_next = widgets.Button(plt.axes([0.33, 0.14, 0.12, 0.06]), 'Next >>')
        self.btn_next.on_clicked(self.next_step)

        self.btn_run = widgets.Button(plt.axes([0.52, 0.14, 0.20, 0.06]), 'Calculate Path', color='lightblue')
        self.btn_run.on_clicked(self.run_simulation)

        self.btn_clear = widgets.Button(plt.axes([0.75, 0.14, 0.12, 0.06]), 'Clear', color='salmon')
        self.btn_clear.on_clicked(self.clear_all)

        # --- Row 2: Bullseye controls ---
        self.btn_bullseye = widgets.Button(
            plt.axes([0.05, 0.06, 0.28, 0.06]),
            '⚠  Simulate Bullseye',
            color='#FFD700'
        )
        self.btn_bullseye.on_clicked(self.simulate_bullseye)

        self.ax_status = plt.axes([0.36, 0.06, 0.56, 0.06])
        self.ax_status.axis('off')
        self.status_text = self.ax_status.text(
            0, 0.5, "Status: ready",
            transform=self.ax_status.transAxes,
            va='center', fontsize=8.5, color='gray',
            wrap=True
        )

        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.redraw()
        plt.show()

    # =========================================================================
    # SMOOTH PATH INTERPOLATION
    # =========================================================================

    def generate_smooth_path(self, raw_path):
        """Convert list of {x,y,d,s} waypoints into dense animation frames."""
        frames = []
        if not raw_path:
            return frames

        for i in range(len(raw_path) - 1):
            p1, p2 = raw_path[i], raw_path[i + 1]
            d1_deg = {0: 90, 2: 0, 4: -90, 6: 180}[p1['d']]
            dx, dy = p2['x'] - p1['x'], p2['y'] - p1['y']

            hx, hy = {0: (0, 1), 2: (1, 0), 4: (0, -1), 6: (-1, 0)}[p1['d']]
            is_reverse = (dx * hx + dy * hy) < -0.1

            if p1['s'] != -1:
                for _ in range(5):
                    frames.append({'x': p1['x'], 'y': p1['y'], 'angle': d1_deg, 's': p1['s']})

            if p1['d'] == p2['d']:                          # Straight
                steps = max(1, int(np.hypot(dx, dy) * 2))
                for t in np.linspace(0, 1, steps, endpoint=False):
                    frames.append({'x': p1['x'] + dx * t, 'y': p1['y'] + dy * t, 'angle': d1_deg, 's': -1})
            else:                                            # Turn arc
                r = 3
                if   p1['d'] == 0: pivot = (p1['x'] - 3, p1['y']) if dx < 0 else (p1['x'] + 3, p1['y'])
                elif p1['d'] == 4: pivot = (p1['x'] + 3, p1['y']) if dx > 0 else (p1['x'] - 3, p1['y'])
                elif p1['d'] == 2: pivot = (p1['x'], p1['y'] + 3) if dy > 0 else (p1['x'], p1['y'] - 3)
                elif p1['d'] == 6: pivot = (p1['x'], p1['y'] - 3) if dy < 0 else (p1['x'], p1['y'] + 3)

                sv = np.array([p1['x'] - pivot[0], p1['y'] - pivot[1]])
                ev = np.array([p2['x'] - pivot[0], p2['y'] - pivot[1]])
                a_s = np.arctan2(sv[1], sv[0])
                a_e = np.arctan2(ev[1], ev[0])

                if abs(a_e - a_s) > np.pi:
                    if a_e > a_s: a_s += 2 * np.pi
                    else:         a_e += 2 * np.pi

                is_ccw = a_e > a_s
                for t in np.linspace(0, 1, 15, endpoint=False):
                    theta = a_s + (a_e - a_s) * t
                    cx = pivot[0] + r * np.cos(theta)
                    cy = pivot[1] + r * np.sin(theta)
                    tangent = theta + (np.pi / 2 if is_ccw else -np.pi / 2)
                    if is_reverse: tangent += np.pi
                    frames.append({'x': cx, 'y': cy, 'angle': np.degrees(tangent), 's': -1})

        last  = raw_path[-1]
        ld    = {0: 90, 2: 0, 4: -90, 6: 180}[last['d']]
        frames.append({'x': last['x'], 'y': last['y'], 'angle': ld, 's': last['s']})
        return frames

    # =========================================================================
    # PLAYBACK
    # =========================================================================

    def toggle_play(self, event):
        if not self.interpolated_path: return
        if self.is_playing:
            self.stop_playback()
        else:
            if self.current_frame >= len(self.interpolated_path) - 1:
                self.current_frame = 0
            self.is_playing = True
            self.btn_play.label.set_text('Pause')
            self.btn_play.color = 'yellow'
            self.timer.start()

    def stop_playback(self):
        self.is_playing = False
        self.btn_play.label.set_text('Play')
        self.btn_play.color = 'lightgreen'
        self.timer.stop()
        self.fig.canvas.draw_idle()

    def play_step(self):
        if self.current_frame < len(self.interpolated_path) - 1:
            self.current_frame += 1
            self.redraw()
        else:
            self.stop_playback()

    def prev_step(self, event):
        self.stop_playback()
        if self.current_frame > 0:
            self.current_frame -= 1
            self.redraw()

    def next_step(self, event):
        self.stop_playback()
        if self.interpolated_path and self.current_frame < len(self.interpolated_path) - 1:
            self.current_frame += 1
            self.redraw()

    # =========================================================================
    # OBSTACLE PLACEMENT (grid clicks)
    # =========================================================================

    def on_click(self, event):
        if event.inaxes != self.ax: return
        x, y = int(event.xdata), int(event.ydata)
        if not (0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE): return
        if x < 4 and y < 4: return     # protect start zone

        idx = next((i for i, o in enumerate(self.obstacles) if o['x'] == x and o['y'] == y), -1)
        if event.button == 3:           # right-click → remove
            if idx != -1:
                self.obstacles.pop(idx)
                self.reset_path()
        if event.button == 1:           # left-click → add / rotate direction
            if idx != -1:
                self.obstacles[idx]['d'] = (self.obstacles[idx]['d'] + 2) % 8
            else:
                if len(self.obstacles) >= 8: return
                self.obstacles.append({"id": len(self.obstacles) + 1, "x": x, "y": y, "d": 0})
            self.reset_path()

    def reset_path(self):
        self.stop_playback()
        for i, obs in enumerate(self.obstacles): obs['id'] = i + 1
        self.raw_path = []
        self.interpolated_path = []
        self.visited_ids = []
        self.cost = 0.0
        self.current_frame = 0
        self.mode = 'normal'
        self.bullseye_obstacle_id = None
        self.bullseye_new_dir     = None
        self.phase1_path          = []
        self.phase2_raw_path      = []
        self._set_status("Ready — place obstacles and click Calculate Path", "gray")
        self.redraw()

    # =========================================================================
    # CALCULATE PATH  (original /path call)
    # =========================================================================

    def run_simulation(self, event):
        if not self.obstacles:
            self._set_status("No obstacles placed.", "red")
            return

        # If we are returning from bullseye recovery, reset mode first
        self.mode = 'normal'
        self.bullseye_obstacle_id = None
        self.bullseye_new_dir     = None
        self.phase1_path          = []
        self.phase2_raw_path      = []

        self._set_status(f"Computing path for {len(self.obstacles)} obstacles...", "orange")
        self.fig.canvas.draw()

        try:
            res = requests.post(API_URL, json={
                "obstacles": self.obstacles,
                "robot_x": 1, "robot_y": 1, "robot_dir": 0
            })
            if res.status_code == 200:
                data = res.json()
                self.raw_path = data['path']
                self.cost     = data['distance']
                self.interpolated_path = self.generate_smooth_path(self.raw_path)
                self.visited_ids = []
                for p in self.raw_path:
                    if p['s'] != -1 and p['s'] not in self.visited_ids:
                        self.visited_ids.append(p['s'])
                self.current_frame = 0
                self.stop_playback()
                self._set_status(
                    f"Path computed — {len(self.visited_ids)} obstacles, cost {self.cost:.1f}. "
                    "Pause playback at an obstacle, then click ⚠ Simulate Bullseye.",
                    "blue"
                )
                self.redraw()
            else:
                self._set_status(f"Server error {res.status_code}: {res.text[:80]}", "red")
        except Exception as e:
            self._set_status(f"Connection failed: {e}", "red")

    # =========================================================================
    # BULLSEYE SIMULATION
    # =========================================================================

    def _current_obstacle_id(self):
        """
        Walk backwards from current_frame to find the most recent SNAP frame.
        Returns obstacle ID or None.
        """
        if not self.interpolated_path: return None
        for i in range(self.current_frame, -1, -1):
            s = self.interpolated_path[i]['s']
            if s != -1:
                return s
        return None

    def _robot_state_at_frame(self):
        """Return {x, y, dir} snapped to grid from current interpolated frame."""
        f = self.interpolated_path[self.current_frame]
        angle = f['angle']
        rounded = int(round(angle / 90) * 90) % 360
        dir_map = {90: 0, 0: 2, 270: 4, 180: 6}
        d = dir_map.get(rounded, 0)
        return {"x": round(f['x']), "y": round(f['y']), "dir": d}

    def _obstacles_visited_before_frame(self):
        """
        Return set of obstacle IDs that received a SNAP strictly BEFORE the
        current frame (i.e. the robot has already dealt with them).
        """
        done = set()
        for i in range(self.current_frame):
            s = self.interpolated_path[i]['s']
            if s != -1:
                done.add(s)
        return done

    def simulate_bullseye(self, event):
        """
        1.  Find which obstacle the robot is currently at.
        2.  Randomise that obstacle's image direction to a NEW direction
            (different from the one we were just at, i.e. different from
            current obstacle direction in self.obstacles).
        3.  Update self.obstacles locally so the arrow redraws immediately.
        4.  Build remaining obstacle list (current obs with new dir + unvisited obs).
        5.  POST to /bullseye — server plans path to correct face + reroutes rest.
        6.  Replace active interpolated_path with the returned full_path so
            Play/Prev/Next immediately follow the recovery path.
        """
        self.stop_playback()

        if not self.interpolated_path:
            self._set_status("No path — click Calculate Path first.", "red")
            return

        obs_id = self._current_obstacle_id()
        if obs_id is None:
            self._set_status("Robot hasn't reached an obstacle yet. Advance playback.", "orange")
            return

        robot = self._robot_state_at_frame()

        # --- Randomise obstacle direction ---
        obs_entry = next((o for o in self.obstacles if o['id'] == obs_id), None)
        if obs_entry is None:
            self._set_status(f"Obstacle {obs_id} not found in obstacle list.", "red")
            return

        old_dir = obs_entry['d']
        all_dirs = [0, 2, 4, 6]
        other_dirs = [d for d in all_dirs if d != old_dir]
        new_dir = random.choice(other_dirs)

        # Update the local obstacle list so the dashboard arrow redraws with new direction
        obs_entry['d'] = new_dir

        print(f"\n[Dashboard] ⚠  BULLSEYE at obstacle {obs_id}")
        print(f"[Dashboard]    Old direction: {DIR_NAMES[old_dir]}  →  New (correct) direction: {DIR_NAMES[new_dir]}")
        print(f"[Dashboard]    Robot at: {robot}")

        # --- Build remaining obstacles list ---
        # Includes: current bullseye obs (with new dir) + all unvisited obs.
        # We always force the bullseye obstacle in, even if the frame counter
        # has already passed its SNAP frames (which would normally mark it visited).
        visited_before = self._obstacles_visited_before_frame()
        visited_before.discard(obs_id)   # bullseye obs is NOT done — we got a bullseye
        remaining = []
        for o in self.obstacles:
            if o['id'] not in visited_before:
                remaining.append({"id": o['id'], "x": o['x'], "y": o['y'], "d": o['d']})

        print(f"[Dashboard]    Remaining obstacles: {[r['id'] for r in remaining]}")

        payload = {
            "obstacle_id":          obs_id,
            "new_direction":        new_dir,
            "robot_x":              robot["x"],
            "robot_y":              robot["y"],
            "robot_dir":            robot["dir"],
            "remaining_obstacles":  remaining,
        }

        self._set_status(
            f"Bullseye! Obs {obs_id} dir: {DIR_NAMES[old_dir]} → {DIR_NAMES[new_dir]}. Computing recovery...",
            "orange"
        )
        self.fig.canvas.draw()

        try:
            res = requests.post(BULLSEYE_URL, json=payload)

            if res.status_code == 200:
                data = res.json()

                # --- Store phase breakdown for visual overlay ---
                self.bullseye_obstacle_id = obs_id
                self.bullseye_new_dir     = new_dir
                self.phase1_path          = data.get('phase1_path', [])
                self.phase2_raw_path      = data.get('phase2_path', [])
                skipped                   = data.get('skipped_obstacle', False)
                p2_dist                   = data.get('phase2_distance', 0.0)
                resolved                  = data.get('resolved_position', {})

                # --- CRITICAL: replace active path with recovery path ---
                # The robot was mid-path; we freeze the already-travelled portion
                # (frames 0..current_frame) and append the new recovery frames.
                already_travelled = self.interpolated_path[:self.current_frame + 1]

                recovery_raw     = data.get('full_path', [])
                recovery_frames  = self.generate_smooth_path(recovery_raw)

                # Stitch: keep travelled history, then play recovery from here
                self.raw_path            = self.raw_path[:self.current_frame + 1] + recovery_raw
                self.interpolated_path   = already_travelled + recovery_frames
                # current_frame stays the same — robot is at the stitch point
                # Playback will continue forward from current_frame into recovery

                self.mode = 'bullseye_recovery'

                unvisited_count = len([r for r in remaining if r['id'] != obs_id])
                status = (
                    f"Recovery planned! Obs {obs_id} → {DIR_NAMES[new_dir]} face. "
                    f"{'Phase1 SKIPPED (blocked). ' if skipped else ''}"
                    f"Rerouting {unvisited_count} remaining obs, cost {p2_dist:.1f}. "
                    f"Press Play to follow recovery path."
                )
                self._set_status(status, "red" if skipped else "darkgreen")

                print(f"[Dashboard]    Phase 1 commands: {data.get('phase1_commands', [])}")
                print(f"[Dashboard]    Phase 2 commands: {data.get('phase2_commands', [])}")
                print(f"[Dashboard]    Resolved pos: {resolved}")

                self.redraw()

            else:
                self._set_status(f"Server error {res.status_code}: {res.text[:100]}", "red")
                # Undo direction change since server rejected the request
                obs_entry['d'] = old_dir
                self.redraw()

        except Exception as e:
            self._set_status(f"Connection failed: {e}", "red")
            obs_entry['d'] = old_dir
            self.redraw()

    def _set_status(self, msg, color="gray"):
        self.status_text.set_text(f"{msg}")
        self.status_text.set_color(color)

    # =========================================================================
    # CLEAR
    # =========================================================================

    def clear_all(self, event):
        self.obstacles = []
        self.reset_path()

    # =========================================================================
    # REDRAW
    # =========================================================================

    def _draw_path_lines(self, raw_path, alpha, cmap):
        """Draw the path lines between consecutive waypoints."""
        for i in range(len(raw_path) - 1):
            p1, p2 = raw_path[i], raw_path[i + 1]
            x1, y1 = p1['x'] + 0.5, p1['y'] + 0.5
            x2, y2 = p2['x'] + 0.5, p2['y'] + 0.5
            color  = cmap(i / max(len(raw_path) - 1, 1))
            if p1['d'] != p2['d']:
                diff = (p2['d'] - p1['d']) % 8
                rad  = -0.3 if diff == 2 else 0.3
                arr  = patches.FancyArrowPatch(
                    (x1, y1), (x2, y2),
                    connectionstyle=f"arc3,rad={rad}",
                    color=color, alpha=alpha, arrowstyle='-', linewidth=2, zorder=1
                )
                self.ax.add_patch(arr)
            else:
                self.ax.plot([x1, x2], [y1, y2], color=color, alpha=alpha, linewidth=2, zorder=1)

    def _draw_ghost_robots(self, raw_path, alpha_multiplier=1.0):
        """Draw semi-transparent robot outlines at SNAP positions."""
        visit_order = 1
        visited_set = set()
        for i, p in enumerate(raw_path):
            is_snap = (p['s'] != -1)
            is_turn = (i < len(raw_path) - 1 and p['d'] != raw_path[i + 1]['d'])
            if not (is_snap or is_turn or i == 0):
                continue

            alpha = (0.30 if is_snap else 0.05) * alpha_multiplier
            style = '-' if is_snap else '--'
            rx, ry = p['x'] - 1.0, p['y'] - 1.0
            self.ax.add_patch(patches.Rectangle(
                (rx, ry), 3, 3, color='gray', alpha=alpha, linestyle=style, zorder=1
            ))

            # Camera marker
            cx, cy, cd = p['x'] + 0.5, p['y'] + 0.5, p['d']
            cam_rects = {
                0: (cx - 0.5, cy + 1.4, 1, 0.2),
                2: (cx + 1.4, cy - 0.5, 0.2, 1),
                4: (cx - 0.5, cy - 1.6, 1, 0.2),
                6: (cx - 1.6, cy - 0.5, 0.2, 1),
            }
            if cd in cam_rects:
                rx2, ry2, rw, rh = cam_rects[cd]
                self.ax.add_patch(patches.Rectangle(
                    (rx2, ry2), rw, rh, color='red', alpha=alpha * 2, zorder=1
                ))

            if is_snap and p['s'] not in visited_set:
                target = next((o for o in self.obstacles if o['id'] == p['s']), None)
                if target:
                    od  = target['d']
                    lx, ly = target['x'] + 0.5, target['y'] + 0.5
                    offsets = {0: (0, -0.5), 2: (-0.5, 0), 4: (0, 1.5), 6: (1.5, 0)}
                    ox, oy  = offsets.get(od, (0, 0))
                    self.ax.text(
                        lx + ox, ly + oy, f"#{visit_order}",
                        color='purple', fontsize=12, fontweight='bold',
                        ha='center', zorder=5
                    )
                visited_set.add(p['s'])
                visit_order += 1

    def redraw(self):
        self.ax.clear()
        self.ax.set_xlim(0, 20)
        self.ax.set_ylim(0, 20)
        self.ax.set_xticks(range(21))
        self.ax.set_yticks(range(21))
        self.ax.grid(True, linestyle=':', alpha=0.6)

        # Title
        mode_tag = "  [BULLSEYE RECOVERY]" if self.mode == 'bullseye_recovery' else ""
        frame_info = (
            f"Frame {self.current_frame}/{len(self.interpolated_path)-1}"
            if self.interpolated_path else "Setup Mode"
        )
        title = f"{frame_info}{mode_tag}\nObstacles: {len(self.obstacles)} | Cost: {self.cost:.2f}"
        if self.raw_path:
            title += f"\nOriginal visit order: {self.visited_ids}"
        if self.mode == 'bullseye_recovery' and self.bullseye_obstacle_id is not None:
            title += (
                f"\nBullseye obs {self.bullseye_obstacle_id}: "
                f"correct face → {DIR_NAMES.get(self.bullseye_new_dir, '?')}"
            )
        self.ax.set_title(title, fontsize=9)

        # Start zone
        self.ax.add_patch(patches.Rectangle((0, 0), 4, 4, color='lightgreen', alpha=0.4))
        self.ax.text(2, 2, "START", ha='center', va='center', color='green', fontweight='bold')

        # ---- Draw obstacles ----
        for obs in self.obstacles:
            x, y, d = obs['x'], obs['y'], obs['d']
            # Bullseye obstacle gets a gold border in recovery mode
            is_bullseye_obs = (
                self.mode == 'bullseye_recovery' and obs['id'] == self.bullseye_obstacle_id
            )
            face_color = '#FFFACD' if is_bullseye_obs else 'salmon'  # lemon chiffon if bullseye
            edge_color = 'gold'    if is_bullseye_obs else 'darkred'
            edge_width = 3         if is_bullseye_obs else 1

            self.ax.add_patch(patches.Rectangle(
                (x, y), 1, 1,
                color=face_color, ec=edge_color, linewidth=edge_width, zorder=2
            ))
            ox, oy, adx, ady = FACE_ARROW[d]
            self.ax.arrow(
                x + ox, y + oy, adx, ady,
                color='green', width=0.08, head_width=0.25, zorder=3
            )
            self.ax.text(
                x + 0.5, y + 0.5, str(obs['id']),
                ha='center', va='center', color='white', fontweight='bold', zorder=4
            )
            if is_bullseye_obs:
                # Gold star to mark it clearly
                self.ax.text(
                    x + 0.5, y + 1.3, "★",
                    ha='center', va='center', color='gold',
                    fontsize=14, fontweight='bold', zorder=5
                )

        # ---- Original path (faded when in recovery) ----
        if self.raw_path:
            orig_alpha = 0.08 if self.mode == 'bullseye_recovery' else 0.30
            # Only draw original path up to current frame's raw waypoint
            self._draw_path_lines(self.raw_path, orig_alpha, plt.cm.winter)
            self._draw_ghost_robots(self.raw_path, alpha_multiplier=(0.3 if self.mode == 'normal' else 0.1))

        # ---- Recovery overlays ----
        if self.mode == 'bullseye_recovery':

            # Phase 1 path — cyan, bold
            if self.phase1_path and len(self.phase1_path) > 1:
                for i in range(len(self.phase1_path) - 1):
                    p1 = self.phase1_path[i]
                    p2 = self.phase1_path[i + 1]
                    x1, y1 = p1['x'] + 0.5, p1['y'] + 0.5
                    x2, y2 = p2['x'] + 0.5, p2['y'] + 0.5
                    self.ax.annotate(
                        '', xy=(x2, y2), xytext=(x1, y1),
                        arrowprops=dict(arrowstyle='->', color='cyan', lw=2.5, alpha=0.9),
                        zorder=6
                    )
                # Label endpoint of phase 1
                end_p = self.phase1_path[-1]
                self.ax.plot(end_p['x'] + 0.5, end_p['y'] + 0.5, 'c*', markersize=14, zorder=7)
                self.ax.text(
                    end_p['x'] + 0.5, end_p['y'] - 0.7,
                    "SNAP ✓", color='cyan', fontsize=8, fontweight='bold',
                    ha='center', zorder=7,
                    bbox=dict(boxstyle='round,pad=0.15', facecolor='navy', alpha=0.7)
                )

            # Phase 2 path — orange/red gradient
            if self.phase2_raw_path and len(self.phase2_raw_path) > 1:
                self._draw_path_lines(self.phase2_raw_path, 0.75, plt.cm.autumn)
                self._draw_ghost_robots(self.phase2_raw_path, alpha_multiplier=0.8)

                # "REROUTE" label at start of phase 2
                sp = self.phase2_raw_path[0]
                self.ax.text(
                    sp['x'] + 0.5, sp['y'] + 1.6,
                    "REROUTE ▶",
                    color='darkorange', fontsize=8, fontweight='bold',
                    ha='center', zorder=7,
                    bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8)
                )

        # ---- Active robot ----
        if self.interpolated_path:
            f  = self.interpolated_path[self.current_frame]
            fx = f['x'] + 0.5
            fy = f['y'] + 0.5
            fa = f['angle']

            t = transforms.Affine2D().rotate_deg_around(fx, fy, fa - 90) + self.ax.transData

            body = patches.Rectangle(
                (fx - 1.5, fy - 1.5), 3, 3,
                color='gray', alpha=0.5, ec='black', lw=3, zorder=10
            )
            body.set_transform(t)
            self.ax.add_patch(body)

            for (bx, by, bw, bh) in [
                (fx - 1.6, fy - 1.2, 0.4, 0.8),   # rear-left wheel
                (fx + 1.2, fy - 1.2, 0.4, 0.8),   # rear-right wheel
                (fx - 1.6, fy + 0.4, 0.4, 0.8),   # front-left wheel
                (fx + 1.2, fy + 0.4, 0.4, 0.8),   # front-right wheel
            ]:
                w = patches.Rectangle((bx, by), bw, bh, color='black', zorder=11)
                w.set_transform(t)
                self.ax.add_patch(w)

            cam = patches.Rectangle((fx - 0.5, fy + 1.4), 1, 0.2, color='red', zorder=11)
            cam.set_transform(t)
            self.ax.add_patch(cam)

            # Direction arrow
            adx = 0.8 * np.cos(np.radians(fa))
            ady = 0.8 * np.sin(np.radians(fa))
            self.ax.arrow(fx, fy, adx, ady, color='blue', width=0.1, head_width=0.3, zorder=11)

            # SNAP indicator
            if f['s'] != -1:
                self.ax.text(fx, fy, "📷", fontsize=14, ha='center', va='center', zorder=12)

            # Gold ring around robot in recovery mode
            if self.mode == 'bullseye_recovery':
                self.ax.add_patch(patches.Circle(
                    (fx, fy), 1.8, fill=False, ec='gold', lw=3, zorder=12, alpha=0.9
                ))

        self.fig.canvas.draw()


if __name__ == "__main__":
    dashboard = InteractiveDashboard()
