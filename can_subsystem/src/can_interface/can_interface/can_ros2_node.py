import os
import can
import struct
import time
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class CanMotorNode(Node):
    def __init__(self):
        super().__init__('can_motor_node')
        self.get_logger().info('Starting CAN Motor Node...')

        # Load motor config from yaml file
        config_path = os.path.join(os.path.expanduser('~'), 'Marsworks', 'motors.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.motors = config['motors']
        self.get_logger().info(f'Loaded {len(self.motors)} motor(s) from config')

        # Connect to CAN bus
        self.bus = can.interface.Bus(channel='can0', interface='socketcan')
        self.get_logger().info('Connected to CAN bus!')

        # Enable all motors
        for motor in self.motors:
            self.enable_motor(motor['node_id'])
            self.get_logger().info(f"Enabled motor: {motor['name']}")

        # Create a publisher and subscriber for each motor
        self.publishers_ = {}
        self.subscribers_ = {}

        for motor in self.motors:
            name = motor['name']

            # Publisher — sends joint state to ROS 2
            self.publishers_[name] = self.create_publisher(
                JointState, f'/{name}/state', 10)

            # Subscriber — receives velocity commands from ROS 2
            self.subscribers_[name] = self.create_subscription(
                Float32, f'/{name}/command',
                lambda msg, m=motor: self.command_callback(msg, m),
                10)

            self.get_logger().info(f'Created topics for {name}')

        # Timer — reads CAN messages every 20ms (50Hz)
        self.create_timer(0.02, self.read_can)

    def enable_motor(self, node_id):
        # First set to IDLE to reset any error state
        can_id = (node_id << 5) | 0x007
        data = struct.pack('<I', 1)  # 1 = IDLE
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(msg)
        time.sleep(0.2)

        # Then set to CLOSED LOOP CONTROL
        data = struct.pack('<I', 8)  # 8 = closed loop
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(msg)
        time.sleep(0.2)
        self.get_logger().info(f'Motor {node_id} in closed loop control!')

    def command_callback(self, msg, motor):
        # Received a velocity command, send it to ODrive over CAN
        node_id = motor['node_id']
        max_vel = motor['max_velocity']

        # Clamp velocity to max so we don't go too fast
        velocity = max(min(msg.data, max_vel), -max_vel)

        can_id = (node_id << 5) | 0x00D
        data = struct.pack('<ff', velocity, 0.0)
        can_msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(can_msg)
        self.get_logger().info(f"Motor {motor['name']}: velocity set to {velocity}")

    def read_can(self):
        # Called every 20ms by the timer
        # Read any waiting CAN messages and publish to ROS 2
        message = self.bus.recv(timeout=0.01)

        if message:
            node_id = message.arbitration_id >> 5
            cmd_id = message.arbitration_id & 0x1F

            # Find which motor this message is from
            motor = next((m for m in self.motors if m['node_id'] == node_id), None)

            if motor and cmd_id == 0x009:
                # Encoder estimates — decode position and velocity
                position, velocity = struct.unpack('<ff', message.data)

                # Publish as JointState message
                joint_msg = JointState()
                joint_msg.header.stamp = self.get_clock().now().to_msg()
                joint_msg.name = [motor['name']]
                joint_msg.position = [float(position)]
                joint_msg.velocity = [float(velocity)]
                joint_msg.effort = [0.0]

                self.publishers_[motor['name']].publish(joint_msg)

    def destroy_node(self):
        # Called when node shuts down — stop all motors safely
        self.get_logger().info('Shutting down — stopping all motors...')
        for motor in self.motors:
            can_id = (motor['node_id'] << 5) | 0x00D
            data = struct.pack('<ff', 0.0, 0.0)
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            self.bus.send(msg)
        self.bus.shutdown()
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()