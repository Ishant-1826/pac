import paho.mqtt.client as mqtt
import json
import time
import subprocess
import os

def test_pivot_at_dist(stop_dist):
    print(f"\n==========================================")
    print(f"Testing pivot at stop_dist = {stop_dist:.3f}m")
    print(f"==========================================")
    subprocess.run(['killall', '-9', 'task_1b_launch'], stderr=subprocess.DEVNULL)
    time.sleep(0.5)

    env = os.environ.copy()
    env['DISPLAY'] = ':1'
    subprocess.Popen(['./task_1b_launch'], env=env)
    time.sleep(1.5)

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    readings = []
    poses = []

    def on_msg(client, u, msg):
        if msg.topic == 'pacbot/sensors':
            readings.append(json.loads(msg.payload.decode()))
        elif msg.topic == 'robot/pose':
            poses.append(json.loads(msg.payload.decode()))

    c.on_message = on_msg
    c.connect('localhost', 1883)
    c.subscribe([('pacbot/sensors', 0), ('robot/pose', 0)])
    c.loop_start()
    time.sleep(1.0)

    # 1. Approach wall until fwd < stop_dist
    print("Approaching wall...")
    for step in range(100):
        s = readings[-1]
        fwd = min(s['sl'], s['sr'])
        if fwd <= stop_dist:
            print(f"Stopped at step {step}: fwd={fwd:.4f} (sl={s['sl']:.4f}, sr={s['sr']:.4f})")
            break
        c.publish('pacbot/wheel_vel', json.dumps({'left': 16.0, 'right': 16.0}))
        time.sleep(0.03)

    c.publish('pacbot/wheel_vel', json.dumps({'left': 0.0, 'right': 0.0}))
    time.sleep(0.3)

    # 2. Pivot 90 degrees right
    print("Pivoting 90 deg right (-20, +20)...")
    yaw_start = poses[-1]['yaw'] if poses else 0.0
    for _ in range(25):
        c.publish('pacbot/wheel_vel', json.dumps({'left': 20.0, 'right': -20.0}))
        time.sleep(0.03)

    c.publish('pacbot/wheel_vel', json.dumps({'left': 0.0, 'right': 0.0}))
    time.sleep(0.5)

    s_after = readings[-1]
    p_after = poses[-1]
    yaw_end = p_after['yaw']
    print(f"After turn: yaw={yaw_end:.1f} (delta={yaw_end - yaw_start:.1f})")
    print(f"  fwd={min(s_after['sl'], s_after['sr']):.4f}, fl={s_after['fl']:.4f}, fr={s_after['fr']:.4f}")
    c.loop_stop()

if __name__ == '__main__':
    for d in [0.085, 0.075, 0.068]:
        test_pivot_at_dist(d)
