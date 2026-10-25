"""PacBot Task 2A - solver boilerplate.
"""
import heapq
import json
import math

import paho.mqtt.client as mqtt

# ------------------------------------------------------------------ MQTT
MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"      
TOPIC_POSES = "pacbot/poses"          
TOPIC_PELLETS = "pacbot/pellets"      
TOPIC_RESULT = "pacbot/result"        
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  

# ------------------------------------------------------------------ maze
N = 15            
PITCH = 0.22      
WALL_HALF = 0.005  
EXIT = (N - 1, N - 1, 3)  

STEP = ((0, 1), (-1, 0), (0, -1), (1, 0))

LAYOUT = (
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",
    "|                           |       |               |       |",
    "+---+---+   +---+   +   +   +   +   +---+   +---+   +---+   +",
    "|           |           |   |   |       |       |           |",
    "+   +   +   +   +---+---+---+   +---+   +   +   +---+---+   +",
    "|   |           |               |   |   |   |   |   |       |",
    "+   +---+---+---+   +---+   +---+   +   +---+   +   +   +---+",
    "|   |               |               |       |   |           |",
    "+   +   +---+---+---+   +   +---+   +---+   +   +---+   +   +",
    "|   |       |           |       |   |       |       |       |",
    "+   +---+   +   +   +---+---+   +   +   +---+---+   +---+   +",
    "|           |   |       |       |   |       |       |   |   |",
    "+   +   +   +   +---+   +   +---+   +---+   +   +---+   +   +",
    "|   |       |       |   |   |   |       |       |       |   |",
    "+   +   +   +---+---+   +   +   +   +   +   +---+   +   +   +",
    "|   |   |           |       |       |           |   |       |",
    "+   +   +---+---+   +   +---+---+   +---+---+   +   +---+---+",
    "|   |   |           |           |       |       |           |",
    "+   +   +   +---+---+---+---+   +   +   +   +---+---+---+   +",
    "|   |   |               |       |   |   |           |       |",
    "+   +---+---+   +---+   +   +---+   +   +---+   +   +   +---+",
    "|   |       |   |           |           |       |   |   |   |",
    "+   +   +   +   +   +---+   +---+---+   +   +---+   +   +   +",
    "|   |   |           |   |               |   |               |",
    "+   +   +---+---+---+   +---+---+---+   +   +---+   +---+   +",
    "|   |       |                   |       |       |       |   |",
    "+   +   +   +   +   +   +---+   +   +---+---+   +---+   +   +",
    "|   |   |       |   |           |   |       |   |       |   |",
    "+   +   +---+---+   +   +   +   +   +---+   +   +---+---+   +",
    "|               |           |               |               |",
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+ E +",
)

def _wall_table():
    walls = {}
    for row in range(N):
        for col in range(N):
            r, c = 2 * row + 1, 4 * col
            walls[row, col, 0] = LAYOUT[r][c + 4] == "|"                
            walls[row, col, 1] = LAYOUT[r - 1][c + 1:c + 4] == "---"    
            walls[row, col, 2] = LAYOUT[r][c] == "|"                    
            walls[row, col, 3] = LAYOUT[r + 1][c + 1:c + 4] == "---"    
    return walls

WALLS = _wall_table()

def cell_centre(cell):
    return cell[1] * PITCH, (N - 1 - cell[0]) * PITCH

def inside(row, col):
    return 0 <= row < N and 0 <= col < N

def neighbours(cell):
    row, col = cell
    out = []
    for d in range(4):
        n = (row + STEP[d][0], col + STEP[d][1])
        if not WALLS[row, col, d] and inside(*n):
            out.append((d, n))
    return out

# ----------------------------------------------------------------- robot
WHEEL_R = 0.017           
TRACK = 0.078             
TICKS_PER_REV = 360       
M_PER_TICK = 2 * math.pi * WHEEL_R / TICKS_PER_REV
WHEEL_MAX = 30.0          

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def wheels(v, omega):
    left = (v - omega * TRACK / 2) / WHEEL_R
    right = (v + omega * TRACK / 2) / WHEEL_R
    
    # CRITICAL ZERO-COLLISION FIX: Proportional scaling instead of independent clamping
    max_wheel = max(abs(left), abs(right))
    if max_wheel > WHEEL_MAX:
        scale_factor = WHEEL_MAX / max_wheel
        left *= scale_factor
        right *= scale_factor
        
    return left, right

# ============================================================== planning

def plan_route(start, heading, pellet):
    def bfs(source, target, init_d):
        initial = (source, init_d)
        dist = {initial: 0.0}
        parent = {}
        queue = [(0.0, source, init_d)]
        goal = None

        while queue:
            cost, cell, direction = heapq.heappop(queue)
            state = (cell, direction)

            if cost != dist.get(state):
                continue

            if cell == target:
                goal = state
                break

            for nd, nxt in neighbours(cell):
                turn = min((nd - direction) % 4, (direction - nd) % 4)
                new_cost = cost + 1.0 + 0.3 * turn
                ns = (nxt, nd)
                if new_cost < dist.get(ns, float("inf")):
                    dist[ns] = new_cost
                    parent[ns] = state
                    heapq.heappush(queue, (new_cost, nxt, nd))

        if goal is None:
            return [], init_d

        directions = []
        cur = goal
        while cur != initial:
            directions.append(cur[1])
            cur = parent[cur]

        directions.reverse()
        return directions, goal[1]

    path_pellet, cur_dir = bfs(tuple(start), tuple(pellet), heading)
    path_exit, cur_dir = bfs(tuple(pellet), (N - 1, N - 1), cur_dir)

    all_dirs = path_pellet + path_exit
    actions = []
    cd = heading

    for d in all_dirs:
        if d != cd:
            actions.append(("turn", d))
            cd = d
        if actions and actions[-1][0] == "run":
            actions[-1] = ("run", actions[-1][1] + 1)
        else:
            actions.append(("run", 1))

    if cd != EXIT[2]:
        actions.append(("turn", EXIT[2]))
    actions.append(("exit", 1))

    return actions

# ================================================================ solver

class Solver:
    def __init__(self):
        self.t = 0.0
        self.spawn = None
        self.pellet = None
        self.actions = None
        self.prev_ticks = None
        self.done = False

        self.theta = 0.0
        self.action_i = 0
        self.progress = 0.0
        self.target_heading = 0.0

    def log(self, text):
        print(f"[{self.t:7.2f}s] {text}", flush=True)

    def on_poses(self, msg):
        bot, d = msg.get("bot"), msg.get("dir")
        if isinstance(bot, list) and d is not None and self.spawn is None:
            self.spawn = (int(d), tuple(bot))
            self.theta = int(d) * (math.pi / 2)
            self.target_heading = self.theta
            self.log(f"spawn {self.spawn[1]} facing {self.spawn[0]}")

    def on_pellets(self, msg):
        pellet = msg.get("pellet")
        if pellet and self.pellet is None:
            self.pellet = tuple(pellet)
            self.log(f"pellet {self.pellet}")

    def step(self, s):
        self.t += s["dt"]
        ticks = (s["ticks_l"], s["ticks_r"])
        if self.prev_ticks is None:
            self.prev_ticks = ticks

        d_left = (ticks[0] - self.prev_ticks[0]) * M_PER_TICK
        d_right = (ticks[1] - self.prev_ticks[1]) * M_PER_TICK
        self.prev_ticks = ticks

        ds = (d_left + d_right) / 2.0
        gyro_dtheta = s["gyro"][2] * s["dt"]

        self.theta = wrap(self.theta + gyro_dtheta)

        if self.done or self.spawn is None or self.pellet is None:
            return 0.0, 0.0

        if self.actions is None:
            self.actions = plan_route(self.spawn[1], self.spawn[0], self.pellet)
            self.log(f"plan: {self.actions}")
            if not self.actions:
                self.done = True
                return 0.0, 0.0

        self.progress += ds

        # Using a while loop seamlessly chains actions without wasting ticks stopping
        while self.action_i < len(self.actions):
            action, val = self.actions[self.action_i]

            if action == "turn":
                self.target_heading = val * (math.pi / 2)
                error = wrap(self.target_heading - self.theta)

                # Tightened tolerance prevents bot from moving forward before fully facing the corridor
                if abs(error) < 0.04: 
                    self.action_i += 1
                    self.progress = 0.0
                    continue

                omega = clamp(20.0 * error, -12.0, 12.0)
                if abs(omega) < 1.5:
                    omega = 1.5 if omega > 0 else -1.5
                return wheels(0.0, omega)

            if action in ("run", "exit"):
                target_dist = (val * PITCH) if action == "run" else (0.85 * PITCH)
                remaining = target_dist - self.progress

                # Tightened distance tolerance forces it to perfectly center before turning
                if remaining <= 0.005: 
                    self.action_i += 1
                    self.progress = 0.0
                    continue

                heading_err = wrap(self.target_heading - self.theta)

                # Narrowed lateral sensor threshold ignores open intersections, preventing swerves
                fl, fr = s["fl"], s["fr"]
                lateral_correction = 0.0
                if fl < 0.14 and fr < 0.14:
                    lateral_correction = 5.0 * (fl - fr) 
                elif fl < 0.08:
                    lateral_correction = -4.0
                elif fr < 0.08:
                    lateral_correction = 4.0

                # Max out Forward Speed (0.51 m/s perfectly utilizes 30.0 rad/s physical sim limit)
                v = clamp(15.0 * remaining, 0.15, 0.51) 
                omega = clamp(8.0 * heading_err + lateral_correction, -5.0, 5.0)
                return wheels(v, omega)

        self.done = True
        return 0.0, 0.0

# ================================================================== main

def _client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()

def main():
    solver = Solver()
    client = _client()

    def send(cli, left, right):
        cli.publish(TOPIC_WHEEL_VEL, json.dumps({"left": float(left), "right": float(right)}))

    def on_message(cli, _userdata, msg):
        if msg.topic == TOPIC_SENSORS:
            send(cli, *solver.step(json.loads(msg.payload)))
        elif msg.topic == TOPIC_POSES:
            if msg.payload:
                solver.on_poses(json.loads(msg.payload))
        elif msg.topic == TOPIC_PELLETS:
            if msg.payload:
                solver.on_pellets(json.loads(msg.payload))
        elif msg.topic == TOPIC_RESULT:
            solver.log(f"result: {msg.payload.decode()}")
            solver.done = True
            send(cli, 0.0, 0.0)
            cli.disconnect()

    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    for topic in (TOPIC_POSES, TOPIC_PELLETS, TOPIC_RESULT, TOPIC_SENSORS):
        client.subscribe(topic)
    print("connected, waiting for the simulator", flush=True)
    client.loop_forever()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass