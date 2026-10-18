import can
import struct
import time

RUN_TIME = 2.0  # seconds to run for — change this to whatever you want

try:
    bus = can.interface.Bus(channel='can0', interface='socketcan')
    print("Connected to CAN bus!")

    # Put ODrive into closed loop control
    can_id = (0 << 5) | 0x007
    data = struct.pack('<I', 8)
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    bus.send(msg)
    print("Closed loop control enabled!")
    time.sleep(0.1)

    # Send velocity command
    can_id_vel = (0 << 5) | 0x00D
    data = struct.pack('<ff', 1.0, 0.0)
    msg = can.Message(arbitration_id=can_id_vel, data=data, is_extended_id=False)
    bus.send(msg)
    print(f"Motor spinning for {RUN_TIME} seconds!")

    # Run for RUN_TIME seconds reading position and velocity
    start = time.time()
    while time.time() - start < RUN_TIME:
        message = bus.recv(timeout=0.1)
        if message:
            cmd_id = message.arbitration_id & 0x1F
            if cmd_id == 0x009:
                position, velocity = struct.unpack('<ff', message.data)
                print(f"Position: {position:.3f} turns | Velocity: {velocity:.3f} turns/s")

except KeyboardInterrupt:
    print("\nStopping early...")

finally:
    # Always stop motor
    can_id = (0 << 5) | 0x00D
    data = struct.pack('<ff', 0.0, 0.0)
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    bus.send(msg)
    print("Motor stopped!")
    bus.shutdown()