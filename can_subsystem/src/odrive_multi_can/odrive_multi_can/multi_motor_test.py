"""Standalone CAN demo, no ROS needed (just python-can + PyYAML).

1 motor (node 0, 1 turn/s for 2 s):
    python3 -m odrive_multi_can.multi_motor_test
Several motors at once:
    python3 -m odrive_multi_can.multi_motor_test --nodes 0 1 2 3 --vel 1.0 --time 3
Or every motor in motors.yaml:
    python3 -m odrive_multi_can.multi_motor_test --config config/motors.yaml
Just listen (no motion) to check wiring / node IDs / bitrate:
    python3 -m odrive_multi_can.multi_motor_test --nodes 0 1 --listen
After colcon build, `ros2 run odrive_multi_can multi_motor_test <same args>` also works.
"""

import argparse
import sys
import time

from odrive_multi_can.motor_manager import MotorConfig, MotorManager, load_config


def parse_args():
    p = argparse.ArgumentParser(description='ODrive CAN demo')
    p.add_argument('--interface', default=None, help='CAN channel (default can0)')
    p.add_argument('--nodes', type=int, nargs='+', help='ODrive node IDs (default 0)')
    p.add_argument('--config', help='motors.yaml to use instead of --nodes')
    p.add_argument('--vel', type=float, default=1.0, help='velocity, turns/s')
    p.add_argument('--time', type=float, default=2.0, help='seconds to run')
    p.add_argument('--listen', action='store_true', help='only print status, do not move')
    return p.parse_args()


def main():
    args = parse_args()
    if args.config:
        cfg = load_config(args.config)
        motors, interface = cfg['motors'], cfg['interface']
    else:
        nodes = args.nodes or [0]
        motors = [MotorConfig(name=f'node{n}', node_id=n, max_velocity=abs(args.vel))
                  for n in nodes]
        interface = 'can0'
    interface = args.interface or interface

    mgr = MotorManager(motors, interface=interface)
    print(f'Connected to {interface}, motors: {[m.name for m in motors]}')
    try:
        missing = mgr.wait_for_heartbeats()
        for name, s in mgr.snapshot().items():
            if name in missing:
                print(f'  {name}: NO HEARTBEAT (check wiring, node_id, bitrate, termination)')
            else:
                print(f'  {name}: state={s.axis_state} error=0x{s.axis_error:08X}')
        if len(missing) == len(motors):
            print('Nothing on the bus answered. Stopping.')
            return 1

        if not args.listen:
            result = mgr.enable_all()
            print('Closed loop:', result)
            if not all(result.values()):
                print('Not every axis entered closed loop. Stopping.')
                return 1

        start = time.monotonic()
        last_print = 0.0
        while time.monotonic() - start < args.time:
            if not args.listen:
                mgr.set_velocities({m.name: args.vel for m in motors})
                mgr.send_setpoints()
            now = time.monotonic()
            if now - last_print >= 0.2:
                last_print = now
                print(' | '.join(
                    f'{n}: pos {s.position:7.3f} vel {s.velocity:6.3f} st {s.axis_state}'
                    + (f' ERR 0x{s.axis_error:X}' if s.axis_error else '')
                    for n, s in mgr.snapshot().items()))
            time.sleep(0.02)  # 50 Hz
    except KeyboardInterrupt:
        print('\nStopping early...')
    finally:
        mgr.shutdown()
        print('All motors stopped and set to IDLE.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
