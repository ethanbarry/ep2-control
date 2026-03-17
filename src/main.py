from pymavlink import mavutil
import time
import math

# Connect to ArduPilot SITL
master = mavutil.mavlink_connection('udp:127.0.0.1:15555')
master.wait_heartbeat()
print("Connected to system")

def set_mode(mode):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

def arm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1,0,0,0,0,0,0
    )
    master.motors_armed_wait()

def takeoff(alt):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,0,0,0,0,alt
    )

def goto_local(x, y, z, yaw=None):
    master.mav.set_position_target_local_ned_send(
        int(time.time()),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        x, y, z,
        0,0,0,
        0,0,0,
        yaw if yaw is not None else 0,
        0
    )

def set_yaw(deg):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        deg,
        20,
        1,
        0,
        0,0,0
    )

# GUIDED mode
set_mode("GUIDED")
time.sleep(2)

# Arm
arm()

# Takeoff to 10 meters
takeoff(10)
time.sleep(2)

# Move 10m forward (north)
goto_local(10, 0, -10)
time.sleep(2)

# Rotate 90° right
set_yaw(90)
time.sleep(2)

# Fly forward 10m (east)
goto_local(10, 10, -10)
time.sleep(2)

# Rotate toward home
yaw_to_home = math.degrees(math.atan2(-10, -10))
set_yaw(yaw_to_home)
time.sleep(2)

# Fly home
goto_local(0, 0, -10)
time.sleep(2)

# Land
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,0,0,0,0,0,0,0
)

print("Mission complete")

'''
from pymavlink import mavutil
import time
"""
This is the source code for the GNC (guidance, navigation and control) module of this operation.
This program is responsible for operating the drone and locating the target based on the information
provided by the DSP (digital signal processing) module.

This module will interface with two primary component:
 1. The flight controller, using serial communication
 2. Then DSP module, using inter-process communication
"""

def main():
    # First, we initialize a connection to the flight controller on the specified port
    conn_str = 'udpin:127.0.0.1:15555'
    conn = mavutil.mavlink_connection(conn_str)

    # Then, we wait for a heartbeat from FC to ensure the connection established successfully 
    conn.wait_heartbeat()
    print(
        'Heartbeat received!\nSystem:\t\t%u\nComponent:\t%u'
        % (conn.target_system, conn.target_component)
    )

    # We need to determine:
    # 1. how to change from controller/radio guided mode to script-guided mode
    # 2. error handling
    # 3. requesting and storing DSP info
    # 4. when to return to base

def get_dsp_data():
    # This function will be responsible for receiving data from the DSP module
    # For now, we will just return a placeholder value
    return "This is a string."

# Entry point into the program
main()

'''