"""ROS 2 wrapper around MotorManager.

Topics (for every motor <name> in motors.yaml):
  /<name>/command   std_msgs/Float32         velocity setpoint, turns/s
  /<name>/state     sensor_msgs/JointState   position (turns), velocity (turns/s)
All motors together:
  /motors/command   sensor_msgs/JointState   name[] + velocity[] -> set all in one go
  /motors/state     sensor_msgs/JointState   every motor in config order

Parameters:
  config_file  path to motors.yaml (default: installed share/odrive_multi_can/config/motors.yaml)
"""

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from odrive_multi_can import odrive_can as oc
from odrive_multi_can.motor_manager import MotorManager, load_config


class CanMotorNode(Node):
    def __init__(self):
        super().__init__('multi_motor_node')

        default_cfg = os.path.join(
            get_package_share_directory('odrive_multi_can'), 'config', 'motors.yaml')
        cfg_path = self.declare_parameter('config_file', default_cfg).value
        cfg = load_config(cfg_path)
        self.get_logger().info(
            f"Loaded {len(cfg['motors'])} motor(s) from {cfg_path} on {cfg['interface']}")

        self.mgr = MotorManager(cfg['motors'], interface=cfg['interface'],
                                bustype=cfg['bustype'], bitrate=cfg['bitrate'],
                                command_timeout=cfg['command_timeout'])
        self.names = list(self.mgr.axes.keys())

        missing = self.mgr.wait_for_heartbeats()
        if missing:
            self.get_logger().warn(f'No heartbeat from: {missing} (check wiring, node IDs, bitrate)')

        for name, ok in self.mgr.enable_all(cfg['input_mode']).items():
            if ok:
                self.get_logger().info(f'{name}: closed loop')
            else:
                self.get_logger().error(f'{name}: did NOT enter closed loop')

        self.state_pubs = {}
        for name in self.names:
            self.state_pubs[name] = self.create_publisher(JointState, f'/{name}/state', 10)
            self.create_subscription(
                Float32, f'/{name}/command',
                lambda msg, n=name: self.mgr.set_velocity(n, msg.data), 10)
        self.all_state_pub = self.create_publisher(JointState, '/motors/state', 10)
        self.create_subscription(JointState, '/motors/command', self.all_command_cb, 10)

        self._last_health = {}
        self.create_timer(1.0 / cfg['control_rate'], self.control_loop)

    def all_command_cb(self, msg: JointState):
        if len(msg.name) != len(msg.velocity):
            self.get_logger().warn('/motors/command needs one velocity per name')
            return
        for name, vel in zip(msg.name, msg.velocity):
            if name in self.mgr.axes:
                self.mgr.set_velocity(name, vel)
            else:
                self.get_logger().warn(f'Unknown motor "{name}"', throttle_duration_sec=5.0)

    def control_loop(self):
        timed_out = self.mgr.send_setpoints()
        if timed_out:
            self.get_logger().warn(f'Command timeout, stopping: {timed_out}')

        snap = self.mgr.snapshot()
        stamp = self.get_clock().now().to_msg()

        all_msg = JointState()
        all_msg.header.stamp = stamp
        for name in self.names:
            s = snap[name]
            m = JointState()
            m.header.stamp = stamp
            m.name = [name]
            m.position = [s.position]
            m.velocity = [s.velocity]
            self.state_pubs[name].publish(m)
            all_msg.name.append(name)
            all_msg.position.append(s.position)
            all_msg.velocity.append(s.velocity)

            # Log only when an axis' error/state changes, not every tick
            health = (s.axis_error, s.axis_state)
            if self._last_health.get(name) != health:
                self._last_health[name] = health
                if s.axis_error:
                    self.get_logger().error(
                        f'{name}: axis_error=0x{s.axis_error:08X} state={s.axis_state}')
                elif s.axis_state != oc.AxisState.CLOSED_LOOP_CONTROL:
                    self.get_logger().warn(f'{name}: axis state is {s.axis_state}')
        self.all_state_pub.publish(all_msg)

    def destroy_node(self):
        self.get_logger().info('Shutting down: stopping all motors and setting IDLE')
        self.mgr.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
