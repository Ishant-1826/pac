
"""Boilerplate for PB Task 2B.

All the plumbing, none of the thinking. It connects to the broker, loads the
official maze, receives the pellet positions for this run, and publishes a
wheel velocity command on every sensor frame. Your job is the one TODO block
in `decide()`.
"""
import json
import math
import itertools
from collections import deque

import paho.mqtt.client as mqtt

import maze_2b
from maze_2b import Maze

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"
TOPIC_PELLETS = "pacbot/pellets"
TOPIC_GHOST = "pacbot/ghost"
TOPIC_RESULT = "pacbot/result"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"

WHEEL_RADIUS = 0.017  # m
HALF_TRACK = 0.039    # m, wheel centre to centreline
WHEEL_LIMIT = 30.0    # rad/s
TICKS_PER_REV = 360
TICK_RAD = 2.0 * math.pi / TICKS_PER_REV

def _mqtt_client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()

def wheels(forward, turn):
    """Differential drive mixer: +turn steers LEFT (counter-clockwise)."""
    return forward - turn, forward + turn

def get_shortest_path(maze, start_cell, end_cell):
    queue = deque([(start_cell, [start_cell])])
    visited = {start_cell}
    while queue:
        current, path = queue.popleft()
        if current == end_cell:
            return path
        for heading, nxt in maze.neighbours(current):
            if nxt not in visited:
                visited.add(nxt)
                queue.append((nxt, path + [nxt]))
    return [start_cell, end_cell]

class Controller:
    def __init__(self, maze):
        self.maze = maze
        self.pellets = []
        self.have_pellets = False
        self.elapsed = 0.0
        self.ghost = None
        self.prev_ticks = None

        # Navigation state
        self.full_path = []
        self.path_index = 0
        self.state = "OBSERVE"  # OBSERVE, TURNING, DRIVING
        self.obs_frames = 0
        self.ghost_loop = set()
        
        # Odometry state
        self.x = -0.15
        self.y = 0.0
        self.theta = 0.0
        self.target_theta = 0.0

        # Movement tracking
        self.distance_traveled_in_cell = 0.0
        self.cell_pitch = maze.pitch

    def on_pellets(self, msg):
        self.pellets = [tuple(c) for c in msg.get("pellets", [])]
        self.have_pellets = True
        print(f"pellets remaining: {self.pellets}")

    def plan_route(self):
        spawn = self.maze.spawn_cell
        exit_c = self.maze.exit_cell
        
        # 1. Identify which pellet the ghost is guarding
        guarded_pellet = None
        min_d = float('inf')
        if self.ghost_loop:
            for p in self.pellets:
                d = min(abs(p[0] - gr) + abs(p[1] - gc) for gr, gc in self.ghost_loop)
                if d < min_d:
                    min_d = d
                    guarded_pellet = p
        
        if guarded_pellet:
            print(f"Identified Guarded Pellet: {guarded_pellet}. Routing to collect it last.")

        best_path = None
        min_len = float('inf')

        # 2. Test all permutations, heavily penalizing any route that does not collect the guarded pellet last
        for perm in itertools.permutations(self.pellets):
            penalty = 1000 if (guarded_pellet and perm[-1] != guarded_pellet) else 0

            current_cell = spawn
            full_route = [spawn]
            total_len = 0
            valid = True
            
            for p in perm:
                seg = get_shortest_path(self.maze, current_cell, p)
                if len(seg) < 2 and current_cell != p:
                    valid = False
                    break
                full_route.extend(seg[1:])
                total_len += len(seg) - 1
                current_cell = p
            
            seg = get_shortest_path(self.maze, current_cell, exit_c)
            if len(seg) < 2 and current_cell != exit_c:
                valid = False
            else:
                full_route.extend(seg[1:])
                total_len += len(seg) - 1

            if valid and (total_len + penalty) < min_len:
                min_len = total_len + penalty
                best_path = full_route

        self.full_path = best_path if best_path else [spawn, exit_c]
        self.path_index = 0
        self.state = "TURNING"
        print(f"Planned optimal path with {len(self.full_path)} waypoints.")

    def decide(self, s):
        self.elapsed += s["dt"]

        left_dist = s["fl"]
        right_dist = s["fr"]
        front_dist = min(s["sl"], s["sr"])
        yaw_rate = s["gyro"][2]
        tl, tr = s["ticks_l"], s["ticks_r"]

        if self.prev_ticks is None or s["dt"] <= 0:
            dl, dr = 0, 0
        else:
            dl, dr = tl - self.prev_ticks[0], tr - self.prev_ticks[1]
        self.prev_ticks = (tl, tr)

        # Update Continuous Odometry
        d_dist = WHEEL_RADIUS * ((dl + dr) / 2.0) * TICK_RAD
        self.theta += yaw_rate * s["dt"]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        self.x += d_dist * math.cos(self.theta)
        self.y += d_dist * math.sin(self.theta)

        if not self.have_pellets:
            return 0.0, 0.0

        # --- PHASE 1: OBSERVATION ---
        # Sit at spawn for 4.0 seconds (400 frames) to map the complete ghost loop.
        # This costs NO time marks because the clock only starts when the robot moves.
        if self.state == "OBSERVE":
            self.obs_frames += 1
            if self.ghost is not None:
                self.ghost_loop.add((self.ghost["row"], self.ghost["col"]))
            
            if self.obs_frames >= 400:
                self.plan_route()
            return 0.0, 0.0

        # --- GHOST AVOIDANCE ---
        if self.ghost is not None and self.path_index < len(self.full_path):
            gx, gy = self.ghost["x"], self.ghost["y"]
            dist_to_ghost = math.hypot(self.x - gx, self.y - gy)
            
            current_cell = self.full_path[self.path_index - 1] if self.path_index > 0 else self.full_path[0]
            target_cell = self.full_path[self.path_index]
            
            in_ghost_zone = current_cell in self.ghost_loop
            entering_ghost_zone = target_cell in self.ghost_loop and not in_ghost_zone
            
            # If we are ABOUT TO ENTER the ghost zone and it is within ~3.5 cells distance, WAIT.
            if entering_ghost_zone and dist_to_ghost < 0.8:
                return 0.0, 0.0
            
            # If we are NOT in the zone but the ghost is extremely close, wait.
            # IMPORTANT: We NEVER freeze if `in_ghost_zone` is True (we drive through fast to survive).
            if not in_ghost_zone and not entering_ghost_zone and dist_to_ghost < 0.25:
                return 0.0, 0.0

        # Check for maze exit
        if self.x >= 3.19:
            return 0.0, 0.0

        if self.path_index >= len(self.full_path):
            return wheels(14.0, 0.0)

        # --- TARGET RESOLUTION ---
        target_cell = self.full_path[self.path_index]
        tx, ty = self.maze.cell_center(target_cell)

        if self.path_index == 0:
            dc, dr_grid = 1, 0
            req_theta = 0.0
        else:
            prev_cell = self.full_path[self.path_index - 1]
            dr_grid = target_cell[0] - prev_cell[0]
            dc = target_cell[1] - prev_cell[1]
            if dc == 1: req_theta = 0.0
            elif dr_grid == -1: req_theta = math.pi / 2.0
            elif dc == -1: req_theta = math.pi
            else: req_theta = -math.pi / 2.0

        # --- PHASE 2: EXECUTION ---
        if self.state == "TURNING":
            angle_err = (req_theta - self.theta + math.pi) % (2 * math.pi) - math.pi
            if abs(angle_err) > 0.04:
                turn_cmd = max(-14.0, min(14.0, 8.0 * angle_err))
                return wheels(0.0, turn_cmd)
            else:
                self.theta = req_theta  # Snap heading exactly to grid angle
                self.state = "DRIVING"
                return wheels(0.0, 0.0)

        elif self.state == "DRIVING":
            self.distance_traveled_in_cell += abs(d_dist)
            target_dist = 0.15 if self.path_index == 0 else self.cell_pitch
            
            # Analyze upcoming turn
            will_turn = False
            new_target_theta = req_theta
            
            if self.path_index < len(self.full_path) - 1:
                cur = self.full_path[self.path_index]
                nxt = self.full_path[self.path_index + 1]
                dr_nxt = nxt[0] - cur[0]
                dc_nxt = nxt[1] - cur[1]
                
                if dc_nxt == 1: new_target_theta = 0.0
                elif dr_nxt == -1: new_target_theta = math.pi / 2.0
                elif dc_nxt == -1: new_target_theta = math.pi
                else: new_target_theta = -math.pi / 2.0
                
                angle_diff = (new_target_theta - req_theta + math.pi) % (2 * math.pi) - math.pi
                if abs(angle_diff) > 0.1:
                    will_turn = True

            # Target Cell Arrival Detection
            reached = False
            if dc == 1 and self.x >= tx: reached = True
            elif dc == -1 and self.x <= tx: reached = True
            elif dr_grid == -1 and self.y >= ty: reached = True
            elif dr_grid == 1 and self.y <= ty: reached = True
            
            if will_turn and front_dist < 0.08:
                reached = True
            elif self.distance_traveled_in_cell >= target_dist:
                reached = True

            if reached:
                self.x = tx
                self.y = ty
                self.path_index += 1
                self.distance_traveled_in_cell = 0.0
                
                if self.path_index >= len(self.full_path):
                    return wheels(14.0, 0.0)
                    
                if will_turn:
                    self.target_theta = new_target_theta
                    self.state = "TURNING"
                    return wheels(0.0, 0.0)
                    
            # Grid Snapping: Erase lateral drift
            if dc != 0:
                self.y = self.y * 0.95 + ty * 0.05
            else:
                self.x = self.x * 0.95 + tx * 0.05

            # Safe Wall Centering
            wall_err = 0.0
            if left_dist < 0.35 and right_dist < 0.35:
                wall_err = left_dist - right_dist
            else:
                if left_dist < 0.09:
                    wall_err = (left_dist - 0.11) * 3.0
                elif right_dist < 0.09:
                    wall_err = (0.11 - right_dist) * 3.0

            angle_err = (req_theta - self.theta + math.pi) % (2 * math.pi) - math.pi
            steering = 4.0 * angle_err + 1.5 * wall_err
            
            dist_to_go = target_dist - self.distance_traveled_in_cell
            if will_turn and dist_to_go < 0.08:
                speed = 4.0
            else:
                speed = 14.0
                
            if front_dist < 0.05:
                speed = 0.0

            return wheels(speed, steering)

        return 0.0, 0.0


def main():
    maze = Maze.official()
    print(f"official maze {maze_2b.MAZE_ID}: {maze.grid}x{maze.grid}, "
          f"spawn row {maze.spawn_row}, exit row {maze.exit_row}")

    controller = Controller(maze)
    client = _mqtt_client()

    def on_message(cli, userdata, msg):
        payload = json.loads(msg.payload.decode())
        if msg.topic == TOPIC_PELLETS:
            controller.on_pellets(payload)
            return
        if msg.topic == TOPIC_GHOST:
            controller.ghost = payload
            return
        if msg.topic == TOPIC_RESULT:
            print(f"run over: solved={payload['solved']}, "
                  f"collisions={payload['collisions']}")
            return
        left, right = controller.decide(payload)
        left = max(-WHEEL_LIMIT, min(WHEEL_LIMIT, float(left)))
        right = max(-WHEEL_LIMIT, min(WHEEL_LIMIT, float(right)))
        cli.publish(TOPIC_WHEEL_VEL,
                    json.dumps({"left": left, "right": right}))

    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(TOPIC_PELLETS)
    client.subscribe(TOPIC_GHOST)
    client.subscribe(TOPIC_RESULT)
    client.subscribe(TOPIC_SENSORS)
    print("connected; waiting for the simulator")
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

