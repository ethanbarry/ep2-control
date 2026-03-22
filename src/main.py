from pymavlink import mavutil
import time

def main():
    # Establish a connection to the drone
    conn = mavutil.mavlink_connection('udpin:localhost:15551')

    # Wait for heartbeat message to confirm connection
    conn.wait_heartbeat()
    print('Heartbeat received.\nSystem ID:\t%d\nComponent ID:\t%d' % (conn.target_system, conn.target_component))

    # Set mode to GUIDED
    conn.mav.set_mode_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # GUIDED mode
    )

    # Await confirmation of mode change
    while True:
        msg = conn.recv_match(type='COMMAND_ACK', blocking=True)
        if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            print('Mode change acknowledged.')
            break
    
    takeoff(25)
    time.sleep(10)
    land()

    def takeoff(target_altitude: int):
        # Arming motors
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm
            0, 0, 0, 0, 0, 0
        )

        # Awaiting arming confirmation
        print("Arming...")
        conn.motors_armed_wait()

        # Send takeoff command
        print(f"Taking off to {target_altitude}m...")
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,          # confirmation
            0, 0, 0, 0, # Params 1-4 (unused)
            0, 0,       # Params 5-6 (Lat/Lon, 0 uses current)
            target_altitude) # Param 7: Target Altitude

    def land():
        print("Landing...")
        conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,          # confirmation
        0, 0, 0, 0, # Params 1-4
        0, 0, 0)    # Params 5-7 (Lat/Lon/Alt, 0 uses current)

    '''
    def goto_position():
        conn.mav.send(mavutil.mavlink.MAVLink_setposition_target_local_ned_message(
            10,     # sender's system time (in milliseconds) since boot
            conn.target_system,     # target_system
            conn.target_component,  # target_component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # coordinate_frame
            int(0b110111111000),              # bitmask to indicate which dimensions should be ignored
            10,     # x position in meters (positive is forward/North)
            0,      # y position in meters (positive is right/East)
            -10,    # z position in meters (positive is down)
            0,      # x velocity in meters/second
            0,      # y velocity in meters/second
            0,      # z velocity in meters/second
            0,      # x acceleration in meters/second^2
            0,      # y acceleration in meters/second^2
            0,      # z acceleration in meters/second^2
            0,      # yaw in radians
            0       # yaw_rate in radians/second
        ))
    '''

