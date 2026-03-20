from typing import List
from algorithms.utils.enums import Direction
from algorithms.utils.types import CellState

class CommandGenerator:
    def generate_commands(self, path: List[CellState]) -> List[str]:
        commands = []
        for i in range(1, len(path)):
            prev = path[i-1]
            curr = path[i]
            
            if prev.direction == curr.direction:
                # --- STRAIGHT MOVEMENT ---
                dist = max(abs(curr.x - prev.x), abs(curr.y - prev.y)) * 10 # Convert cells to cm
                if dist == 0: continue # Skip if no movement (e.g. start node duplicate)
                
                # Check Forward or Backward
                is_forward = False
                if prev.direction == Direction.NORTH and curr.y > prev.y: is_forward = True
                elif prev.direction == Direction.SOUTH and curr.y < prev.y: is_forward = True
                elif prev.direction == Direction.EAST and curr.x > prev.x: is_forward = True
                elif prev.direction == Direction.WEST and curr.x < prev.x: is_forward = True
                
                cmd = "FW" if is_forward else "BW"
                commands.append(f"{cmd}10") 
                
            else:
                # --- TURNING (90 Degree) ---
                diff = (int(curr.direction) - int(prev.direction)) % 8
                
                # Determine if the robot moved forward or backward during the turn
                is_forward_turn = True
                if prev.direction == Direction.NORTH and curr.y < prev.y: is_forward_turn = False
                elif prev.direction == Direction.SOUTH and curr.y > prev.y: is_forward_turn = False
                elif prev.direction == Direction.EAST and curr.x < prev.x: is_forward_turn = False
                elif prev.direction == Direction.WEST and curr.x > prev.x: is_forward_turn = False
                
                if diff == 2: # Nose swings Right
                    commands.append("FR90" if is_forward_turn else "BL90")
                elif diff == 6: # Nose swings Left
                    commands.append("FL90" if is_forward_turn else "BR90")
                elif diff == 4: # 180 Turn (Rare)
                    commands.append("FR90")
                    commands.append("FR90")
                    is_forward_turn = True 
            
            # --- SNAPSHOT ---
            if curr.screenshot_id != -1:
                commands.append(f"SNAP{curr.screenshot_id}")
                
        commands.append("FIN")
        return self.compress_commands(commands)

    def compress_commands(self, commands: List[str]) -> List[str]:
        compressed = []
        if not commands: return []
        
        curr_cmd = commands[0]
        curr_val = 0
        
        def parse(c):
            if c.startswith("FW") or c.startswith("BW"):
                return c[:2], int(c[2:])
            return c, 0

        for i in range(len(commands)):
            cmd = commands[i]
            type_str, val = parse(cmd)
            
            if i == 0:
                curr_val = val
                continue
            
            prev_type, _ = parse(commands[i-1])
            
            if type_str == prev_type and val > 0: # Mergeable (e.g., FW04 + FW10 = FW14)
                curr_val += val
            else:
                # Flush previous
                prev_real_type, _ = parse(commands[i-1])
                if prev_real_type in ["FW", "BW"]:
                    while curr_val > 90:
                        compressed.append(f"{prev_real_type}90")
                        curr_val -= 90
                    if curr_val > 0:
                        compressed.append(f"{prev_real_type}{curr_val:02d}")
                else:
                    compressed.append(commands[i-1])
                
                curr_val = val
                
        # Flush last
        last_type, _ = parse(commands[-1])
        if last_type in ["FW", "BW"]:
             while curr_val > 90:
                compressed.append(f"{last_type}90")
                curr_val -= 90
             if curr_val > 0:
                compressed.append(f"{last_type}{curr_val:02d}")
        else:
            compressed.append(commands[-1])
            
        return compressed