from pymavlink import mavutil
import time

# 1. Connect to the Vehicle
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600) # Example for SITL
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

# 2. Define the Rotate Function
def set_yaw(angle, speed=10, direction=1, relative=1):
    """
    Rotates the drone to a specific yaw angle.
    :param angle: Angle in degrees
    :param speed: Speed of rotation in deg/s
    :param direction: 1 for CW, -1 for CCW
    :param relative: 1 for relative to current, 0 for absolute
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # Command ID: 115
        0,                                     # Confirmation
        angle,                                 # Param 1: Target Angle (deg)
        speed,                                 # Param 2: Speed (deg/s)
        direction,                             # Param 3: Direction (1=CW, -1=CCW)
        relative,                              # Param 4: Relative/Absolute
        0, 0, 0                                # Param 5-7: Unused
    )
    print(f"Sent command to rotate {angle} degrees (Relative: {relative})")

# 3. Execute the rotation (30 degrees clockwise)
set_yaw(30, speed=15, direction=1, relative=1)

# Allow time for the command to execute
time.sleep(5)
