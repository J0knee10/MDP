import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.widgets as widgets
import requests
import numpy as np

# CONFIGURATION
API_URL = "http://localhost:5000/path"
GRID_SIZE = 20

class InteractiveDashboard:
    def __init__(self):
        self.obstacles = []
        self.path = []
        self.cost = 0
        
        # Setup Figure
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        plt.subplots_adjust(bottom=0.2)
        
        # Buttons
        self.ax_run = plt.axes([0.35, 0.05, 0.3, 0.075])
        self.btn_run = widgets.Button(self.ax_run, 'Calculate Path', color='lightblue', hovercolor='skyblue')
        self.btn_run.on_clicked(self.run_simulation)
        
        self.ax_clear = plt.axes([0.7, 0.05, 0.15, 0.075])
        self.btn_clear = widgets.Button(self.ax_clear, 'Clear All', color='salmon', hovercolor='red')
        self.btn_clear.on_clicked(self.clear_all)

        # Mouse Events
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        self.redraw()
        plt.show()

    def on_click(self, event):
        if event.inaxes != self.ax: return
        x, y = int(event.xdata), int(event.ydata)
        if not (0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE): return
        
        # Start Zone Protection
        if x < 4 and y < 4:
            print("❌ Cannot place obstacles in Start Zone!")
            return

        existing_idx = next((i for i, o in enumerate(self.obstacles) if o['x'] == x and o['y'] == y), -1)
        
        # RIGHT CLICK: Remove
        if event.button == 3: 
            if existing_idx != -1:
                self.obstacles.pop(existing_idx)
                for i, obs in enumerate(self.obstacles): obs['id'] = i + 1
                self.path = []
                self.cost = 0
                self.redraw()
            return

        # LEFT CLICK: Add or Rotate
        if event.button == 1:
            if existing_idx != -1:
                self.obstacles[existing_idx]['d'] = (self.obstacles[existing_idx]['d'] + 2) % 8
            else:
                if len(self.obstacles) >= 8: return
                self.obstacles.append({"id": len(self.obstacles) + 1, "x": x, "y": y, "d": 0})
            
            self.path = []
            self.cost = 0
            self.redraw()

    def run_simulation(self, event):
        if not self.obstacles: return
        print(f"📡 Sending {len(self.obstacles)} obstacles...")
        try:
            payload = {"obstacles": self.obstacles, "robot_x": 1, "robot_y": 1, "robot_dir": 0}
            res = requests.post(API_URL, json=payload)
            if res.status_code == 200:
                data = res.json()
                self.path = data['path']
                self.cost = data['distance']
                print(f"✅ Path found! Cost: {self.cost}")
                self.redraw()
            else:
                print(f"❌ Server Error: {res.text}")
        except Exception as e:
            print(f"❌ Connection Failed: {e}")

    def clear_all(self, event):
        self.obstacles = []
        self.path = []
        self.cost = 0
        self.redraw()

    def redraw(self):
        self.ax.clear()
        self.ax.set_xlim(0, 20); self.ax.set_ylim(0, 20)
        self.ax.set_xticks(range(21)); self.ax.set_yticks(range(21))
        self.ax.grid(True, linestyle=':', alpha=0.6)
        self.ax.set_title(f"L-Click: Place/Rotate | R-Click: Delete\nObstacles: {len(self.obstacles)} | Cost: {self.cost:.2f}")

        # Start Zone
        self.ax.add_patch(patches.Rectangle((0, 0), 4, 4, color='lightgreen', alpha=0.4))
        self.ax.text(2, 2, "START", ha='center', va='center', color='green', fontweight='bold')

        # Draw Obstacles
        for obs in self.obstacles:
            x, y, d = obs['x'], obs['y'], obs['d']
            self.ax.add_patch(patches.Rectangle((x, y), 1, 1, color='salmon', ec='darkred', zorder=2))
            
            # Image Face Arrow (Green)
            face_map = {0:(0.5,1,0,0.3), 2:(1,0.5,0.3,0), 4:(0.5,0,0,-0.3), 6:(0,0.5,-0.3,0)}
            ox, oy, dx, dy = face_map[d]
            self.ax.arrow(x+ox, y+oy, dx, dy, color='green', width=0.08, head_width=0.25, zorder=3)
            self.ax.text(x+0.5, y+0.5, str(obs['id']), ha='center', va='center', color='white', fontweight='bold')

        # --- PATH VISUALIZATION ---
        if self.path:
            # 1. Visiting Order Labels
            visit_order = 1
            visited_ids = set()
            
            for p in self.path:
                sid = p['s']
                if sid != -1 and sid not in visited_ids:
                    target_obs = next((o for o in self.obstacles if o['id'] == sid), None)
                    if target_obs:
                        od = target_obs['d']
                        lx, ly = target_obs['x'] + 0.5, target_obs['y'] + 0.5 
                        
                        # Labels opposite to face
                        if od == 0:   ly = target_obs['y'] - 0.4
                        elif od == 2: lx = target_obs['x'] - 0.4
                        elif od == 4: ly = target_obs['y'] + 1.4
                        elif od == 6: lx = target_obs['x'] + 1.4

                        self.ax.text(lx, ly, f"#{visit_order}", 
                                   color='purple', fontsize=14, fontweight='bold', ha='center', va='center', zorder=5)
                        
                        visited_ids.add(sid)
                        visit_order += 1

            # 2. Path Segments
            for i in range(len(self.path) - 1):
                p1, p2 = self.path[i], self.path[i+1]
                color = plt.cm.winter(i / len(self.path))
                
                # Turn Logic
                if p1['d'] != p2['d']:
                    # Calculate Turn Direction
                    # Right Turn (Diff = 2 or -6) -> Needs negative rad
                    # Left Turn  (Diff = 6 or -2) -> Needs positive rad
                    diff = (p2['d'] - p1['d']) % 8
                    
                    if diff == 2:   # Right Turn
                        rad = -0.3
                    elif diff == 6: # Left Turn
                        rad = 0.3
                    else:           # U-Turn or other (shouldn't happen often)
                        rad = 0.3

                    arrow = patches.FancyArrowPatch(
                        (p1['x'], p1['y']), (p2['x'], p2['y']),
                        connectionstyle=f"arc3,rad={rad}",  # DYNAMIC CURVATURE
                        color=color, arrowstyle='-', linewidth=2, zorder=1
                    )
                    self.ax.add_patch(arrow)
                else:
                    # Straight Line
                    self.ax.plot([p1['x'], p2['x']], [p1['y'], p2['y']], 
                               color=color, linewidth=2, zorder=1)
                    
                    if i % 5 == 0 and i > 0: 
                        mid_x, mid_y = (p1['x'] + p2['x']) / 2, (p1['y'] + p2['y']) / 2
                        dx, dy = (p2['x'] - p1['x']), (p2['y'] - p1['y'])
                        if np.hypot(dx, dy) > 0:
                            self.ax.arrow(mid_x - dx*0.1, mid_y - dy*0.1, dx*0.001, dy*0.001, 
                                        color=color, head_width=0.4, head_length=0.4, zorder=2)

            # 3. Ghosts & Lens
            for i, p in enumerate(self.path):
                is_turn = (i < len(self.path)-1) and (p['d'] != self.path[i+1]['d'])
                is_snapshot = (p['s'] != -1)
                is_start = (i == 0)

                if is_snapshot or is_start or is_turn:
                    # Style
                    alpha = 0.4 if is_snapshot else 0.05
                    style = '-' if is_snapshot else '--'
                    lens_alpha = 1.0 if is_snapshot else 0.3

                    # Body
                    rx, ry = p['x'] - 1.5, p['y'] - 1.5
                    self.ax.add_patch(patches.Rectangle((rx, ry), 3, 3, 
                                                      color='gray', alpha=alpha, linestyle=style))
                    
                    # Lens
                    cx, cy, cd = p['x'], p['y'], p['d']
                    if cd == 0: rect = (cx-0.5, cy+1.4, 1, 0.2)
                    elif cd == 2: rect = (cx+1.4, cy-0.5, 0.2, 1)
                    elif cd == 4: rect = (cx-0.5, cy-1.6, 1, 0.2)
                    elif cd == 6: rect = (cx-1.6, cy-0.5, 0.2, 1)
                    self.ax.add_patch(patches.Rectangle(rect[:2], rect[2], rect[3], color='red', alpha=lens_alpha))

                    if is_snapshot:
                        self.ax.text(p['x'], p['y'], "📷", fontsize=12, ha='center', va='center')
                    elif is_start:
                        self.ax.text(p['x'], p['y'], "S", fontsize=9, ha='center', va='center', color='green', weight='bold')

        self.fig.canvas.draw()

if __name__ == "__main__":
    dashboard = InteractiveDashboard()