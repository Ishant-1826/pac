
"""PB Task 1B - fast right-wall-following PacBot controller.

Uses only the task sensor stream and publishes wheel velocities over MQTT.
Designed to minimize unnecessary slowdowns while retaining conservative
front-obstacle handling and reliable turn completion.
"""
import collections
import json
import math
import sys
import time

import paho.mqtt.client as mqtt

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"

# ---------------------------------------------------------------------------
# Practical high-speed operating band
# ---------------------------------------------------------------------------
# The earlier controller was already requesting very large values.  This
# controller therefore focuses on eliminating time spent in slow states and
# on keeping the command near the simulator's observed useful operating band.
CRUISE_SPEED = 24.0       # maximum practical forward command used by control
APPROACH_SPEED = 22.0     # short corner-approach speed
TURN_SPEED = 24.0         # maximum practical pivot command
UTURN_SPEED = 24.0
MAX_WHEEL_SPEED = 24.0

# ---------------------------------------------------------------------------
# Sensor geometry / thresholds
# ---------------------------------------------------------------------------
SIDE_WALL_PRESENT = 0.18
SIDE_WALL_OPEN = 0.30
TARGET_SIDE_DIST = 0.095

# Close front clearance.  The two forward readings are combined with min().
FRONT_TURN_DIST = 0.108
FRONT_HARD_STOP = 0.075

# Junction confirmation.  Keep this short; the simulator is high-rate.
OPEN_CONFIRM_TICKS = 2

# ---------------------------------------------------------------------------
# Wall PD
# ---------------------------------------------------------------------------
KP = 28.0
KD = 1.6
K_GYRO = 0.30
MAX_STEER = 5.0

# ---------------------------------------------------------------------------
# Turns
# ---------------------------------------------------------------------------
R90 = math.pi / 2.0
U180 = math.pi
TURN_LEAD = 0.045
TURN_TIMEOUT_90 = 1.20
TURN_TIMEOUT_180 = 2.20

# ---------------------------------------------------------------------------
# Exit detection
# ---------------------------------------------------------------------------
EXIT_MIN_RUN_TIME = 1.5
EXIT_OPEN_DISTANCE = 0.65
EXIT_CONFIRM_TIME = 0.08

S_START = "START"
S_FOLLOW = "FOLLOW"
S_ADVANCE = "ADVANCE"
S_DECIDE = "DECIDE"
S_TURN_R = "TURN_R"
S_TURN_L = "TURN_L"
S_TURN_U = "TURN_U"
S_COMPLETE = "COMPLETE"


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def sanitize(x):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return 5.0
    if not math.isfinite(x) or x <= 0.0 or x > 5.0:
        return 5.0
    return x


class PacBotController:
    def __init__(self):
        self.state = S_START
        self.tick = 0
        self.run_start = time.monotonic()
        self.last_control_t = None

        self.fl_hist = collections.deque(maxlen=3)
        self.fr_hist = collections.deque(maxlen=3)
        self.sl_hist = collections.deque(maxlen=3)
        self.sr_hist = collections.deque(maxlen=3)

        self.fl = self.fr = self.sl = self.sr = 5.0
        self.front = 5.0
        self.gz = 0.0

        self.prev_error = 0.0
        self.turn_angle = 0.0
        self.turn_target = 0.0
        self.turn_started = 0.0
        self.turn_dir = 1

        self.right_seen = 0
        self.advance_ticks = 0
        self.exit_open_since = None
        self.maze_seen = False

        self.last_log = 0.0

    @staticmethod
    def median_update(buf, value):
        buf.append(sanitize(value))
        vals = sorted(buf)
        return vals[len(vals) // 2]

    def transition(self, state, why=""):
        if self.state != state:
            if why:
                print(f"[{self.state} -> {state}] {why}", flush=True)
            self.state = state
            self.tick = 0

    def read_sensors(self, data):
        self.fl = self.median_update(self.fl_hist, data.get("fl"))
        self.fr = self.median_update(self.fr_hist, data.get("fr"))
        self.sl = self.median_update(self.sl_hist, data.get("sl"))
        self.sr = self.median_update(self.sr_hist, data.get("sr"))
        self.front = min(self.sl, self.sr)

        gyro = data.get("gyro")
        try:
            self.gz = float(gyro[2]) if gyro and len(gyro) >= 3 else 0.0
        except (TypeError, ValueError):
            self.gz = 0.0
        if not math.isfinite(self.gz):
            self.gz = 0.0

        dt = data.get("dt", None)
        try:
            dt = float(dt)
        except (TypeError, ValueError):
            dt = 0.002
        if not math.isfinite(dt) or dt <= 0.0 or dt > 0.05:
            # Physics timestep is authoritative; use a conservative fallback.
            dt = 0.002
        return dt

    def wall_pd(self, dt):
        has_r = self.fr < SIDE_WALL_PRESENT
        has_l = self.fl < SIDE_WALL_PRESENT

        if has_r and has_l:
            # Match distances instead of aiming at the absolute target when
            # both walls are visible; this keeps the body centered.
            error = self.fr - self.fl
        elif has_r:
            error = self.fr - TARGET_SIDE_DIST
        elif has_l:
            error = TARGET_SIDE_DIST - self.fl
        else:
            error = 0.0

        derivative = (error - self.prev_error) / dt if dt > 1e-5 else 0.0
        # Limit derivative contribution so a one-frame ToF spike cannot
        # generate a large steering impulse at high speed.
        derivative = clamp(derivative, -2.5, 2.5)
        self.prev_error = error

        correction = KP * error + KD * derivative - K_GYRO * self.gz
        return clamp(correction, -MAX_STEER, MAX_STEER)

    def check_exit(self, now):
        if not self.maze_seen:
            return False
        if now - self.run_start < EXIT_MIN_RUN_TIME:
            return False
        clear = (
            self.fl > EXIT_OPEN_DISTANCE
            and self.fr > EXIT_OPEN_DISTANCE
            and self.front > EXIT_OPEN_DISTANCE
        )
        if clear:
            if self.exit_open_since is None:
                self.exit_open_since = now
            return (now - self.exit_open_since) >= EXIT_CONFIRM_TIME
        self.exit_open_since = None
        return False

    def update(self, data):
        self.tick += 1
        dt = self.read_sensors(data)
        now = time.monotonic()

        # Mark that we have actually entered a wall-bounded maze region.
        if self.fl < SIDE_WALL_PRESENT or self.fr < SIDE_WALL_PRESENT:
            self.maze_seen = True

        # ---------------------------------------------------------------
        # START
        # ---------------------------------------------------------------
        if self.state == S_START:
            corr = self.wall_pd(dt)
            left = CRUISE_SPEED + corr
            right = CRUISE_SPEED - corr

            # Avoid making a special slow entry state longer than necessary.
            if self.tick >= 4:
                self.transition(S_FOLLOW, "rapid entry")
            return self.finalize(left, right)

        # ---------------------------------------------------------------
        # COMPLETE
        # ---------------------------------------------------------------
        if self.state == S_COMPLETE:
            return 0.0, 0.0

        # ---------------------------------------------------------------
        # FOLLOW
        # ---------------------------------------------------------------
        if self.state == S_FOLLOW:
            if self.check_exit(now):
                self.transition(S_COMPLETE, "GOAL REACHED")
                return 0.0, 0.0

            # Hard emergency protection.
            if self.front <= FRONT_HARD_STOP:
                self.transition(S_DECIDE, f"hard front stop: {self.front:.3f}m")
                return 0.0, 0.0

            # Front approach: keep moving fast until the turn decision is
            # necessary; do not enter a long slow crawl.
            if self.front <= FRONT_TURN_DIST:
                self.transition(S_DECIDE, f"front close: {self.front:.3f}m")
                return 0.0, 0.0

            # Right-wall rule.  A right opening is confirmed using only a
            # couple of consecutive high-rate samples.
            if self.fr < SIDE_WALL_PRESENT:
                self.right_seen = min(self.right_seen + 1, 20)
            elif self.right_seen >= OPEN_CONFIRM_TICKS and self.fr > SIDE_WALL_OPEN:
                self.right_seen = 0
                self.advance_ticks = 0
                self.transition(S_ADVANCE, f"right opening: {self.fr:.3f}m")
                return self.finalize(APPROACH_SPEED, APPROACH_SPEED)

            corr = self.wall_pd(dt)

            # Keep maximum forward command almost all the way through a clear
            # corridor. Only the final approach bands reduce speed.
            if self.front < 0.15:
                base = 18.0
            elif self.front < 0.22:
                base = 21.0
            else:
                base = CRUISE_SPEED

            return self.finalize(base + corr, base - corr)

        # ---------------------------------------------------------------
        # ADVANCE: just enough forward movement to place the pivot at the
        # opening, without a long slow segment.
        # ---------------------------------------------------------------
        if self.state == S_ADVANCE:
            self.advance_ticks += 1
            if self.front <= FRONT_HARD_STOP or self.advance_ticks >= 2:
                self.turn_angle = 0.0
                self.turn_target = R90
                self.turn_started = now
                self.turn_dir = 1
                self.transition(S_TURN_R, "pivot at right opening")
                return 0.0, 0.0
            return self.finalize(APPROACH_SPEED, APPROACH_SPEED)

        # ---------------------------------------------------------------
        # DECIDE: right -> straight -> left -> U-turn
        # ---------------------------------------------------------------
        if self.state == S_DECIDE:
            r_open = self.fr > SIDE_WALL_OPEN
            f_open = self.front > 0.18
            l_open = self.fl > SIDE_WALL_OPEN

            self.turn_angle = 0.0
            self.turn_target = 0.0
            self.turn_started = now

            if r_open:
                self.turn_target = R90
                self.turn_dir = 1
                self.transition(S_TURN_R, f"choose RIGHT r={self.fr:.3f}")
                return 0.0, 0.0
            if f_open:
                self.right_seen = 0
                self.transition(S_FOLLOW, "straight remains open")
                return self.finalize(CRUISE_SPEED, CRUISE_SPEED)
            if l_open:
                self.turn_target = R90
                self.turn_dir = -1
                self.transition(S_TURN_L, f"choose LEFT l={self.fl:.3f}")
                return 0.0, 0.0

            self.turn_target = U180
            # Pivot away from the closer side.
            self.turn_dir = 1 if self.fr >= self.fl else -1
            self.transition(S_TURN_U, "dead end -> U-turn")
            return 0.0, 0.0

        # ---------------------------------------------------------------
        # TURN states. PID wall correction is disabled during the pivot.
        # ---------------------------------------------------------------
        if self.state in (S_TURN_R, S_TURN_L, S_TURN_U):
            self.turn_angle += abs(self.gz) * dt
            timeout = TURN_TIMEOUT_180 if self.state == S_TURN_U else TURN_TIMEOUT_90
            done = (
                self.turn_angle >= max(0.0, self.turn_target - TURN_LEAD)
                or (now - self.turn_started) >= timeout
            )

            if self.turn_dir > 0:
                left, right = TURN_SPEED, -TURN_SPEED
            else:
                left, right = -TURN_SPEED, TURN_SPEED

            if done:
                deg = math.degrees(self.turn_angle)
                self.prev_error = 0.0
                self.right_seen = 0
                self.transition(S_FOLLOW, f"turn complete {deg:.1f} deg")
                # Resume moving immediately on the next sensor callback.
                return self.finalize(CRUISE_SPEED, CRUISE_SPEED)

            if self.state == S_TURN_U:
                return self.finalize(UTURN_SPEED * self.turn_dir,
                                    -UTURN_SPEED * self.turn_dir)
            return self.finalize(left, right)

        return 0.0, 0.0

    def finalize(self, left, right):
        left = clamp(float(left), -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        right = clamp(float(right), -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        if not (math.isfinite(left) and math.isfinite(right)):
            return 0.0, 0.0

        now = time.monotonic()
        if now - self.last_log >= 0.5:
            self.last_log = now
            print(
                f"[{self.state:<8}] front={self.front:.3f} "
                f"L={self.fl:.3f} R={self.fr:.3f} "
                f"cmd=({left:+.1f},{right:+.1f})",
                flush=True,
            )
        return left, right


controller = PacBotController()


def mqtt_client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def on_message(client, userdata, msg):
    if msg.topic != TOPIC_SENSORS:
        return

    try:
        data = json.loads(msg.payload.decode())
    except Exception as exc:
        print(f"sensor JSON error: {exc}", flush=True)
        return

    left, right = controller.update(data)
    client.publish(
        TOPIC_WHEEL_VEL,
        json.dumps({"left": left, "right": right}, separators=(",", ":")),
    )

    if controller.state == S_COMPLETE:
        print("=== GOAL REACHED - STOPPING AND TERMINATING ===", flush=True)
        client.publish(
            TOPIC_WHEEL_VEL,
            json.dumps({"left": 0.0, "right": 0.0}, separators=(",", ":")),
        )
        # loop_stop is intentionally followed by disconnect and process exit;
        # the goal callback must not leave loop_forever() running.
        try:
            client.loop_stop()
        except Exception:
            pass
        try:
            client.disconnect()
        except Exception:
            pass
        raise SystemExit(0)


def main():
    client = mqtt_client()
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(TOPIC_SENSORS, 0)
    print("=== PacBot Task 1B FAST controller STARTED ===", flush=True)
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nController stopped.", flush=True)
        sys.exit(0)
