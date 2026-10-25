import paho.mqtt.client as mqtt
import json
import time
import subprocess
import os

def main():
    print("Testing forward approach to wall...")
    # Kill any existing task_1b_launch
    subprocess.run(['killall', '-9', 'task_1b_launch'], stderr=subprocess.DEVNULL)
    time.sleep(0.5)

    env = os.environ.copy()
    env['DISPLAY'] = ':1'
    sim = subprocess.Popen(['./task_1b_launch'], env=env)
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
    print("Initial spawn:")
    print(" ", readings[-1])

    print("\nApproaching wall...")
    for step in range(60):
        s = readings[-1]
        fwd = min(s['sl'], s['sr'])
        print(f"step {step:2d}: fwd={fwd:.4f} (sl={s['sl']:.4f}, sr={s['sr']:.4f})  fl={s['fl']:.4f}  fr={s['fr']:.4f}")
        if fwd < 0.080:
            print("Reached fwd < 0.080!")
            break
        c.publish('pacbot/wheel_vel', json.dumps({'left': 12.0, 'right': 12.0}))
        time.sleep(0.03)

    c.publish('pacbot/wheel_vel', json.dumps({'left': 0.0, 'right': 0.0}))
    time.sleep(0.5)
    s_end = readings[-1]
    print("\nFINAL SENSOR AT WALL:")
    print(f"  fwd={min(s_end['sl'], s_end['sr']):.4f}")
    print(f"  fl (LEFT)={s_end['fl']:.4f}")
    print(f"  fr (RIGHT)={s_end['fr']:.4f}")

    c.loop_stop()

if __name__ == '__main__':
    main()
