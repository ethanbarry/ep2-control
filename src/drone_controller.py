from pymavlink import mavutil
import time

class drone_controller:

    def __init__(self, connection_str: str):
        # Estabilish a connection to the drone
        self.conn = mavutil.mavlink_connection(connection_str)
        print('Initializing connection to drone...')

        # Wait for heartbeat message to confirm connection
        self.conn.wait_heartbeat()
        print(
            'Heartbeat received.\nSystem ID:\t%d\nComponent ID:\t%d'
            %
            (self.conn.target_system, self.conn.target_component)
        )

        # Giving the drone a chance to ready
        print('Readying drone...')
        time.sleep(5)
        print('Ready!')

    def set_mode(self, mode: str):
        # Set mode to specified mode (e.g., "GUIDED", "LOITER", "LAND")
        mode_id = self.conn.mode_mapping()[mode]
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f'Setting mode to {mode}...')

        # Await confirmation of mode change
        while True:
            ack = self.conn.recv_match(type='HEARTBEAT', blocking=True)
            if ack.custom_mode == mode_id:
                print(f"System is now in {mode} mode")
                break
            else:
                print(f"Current mode: {ack.custom_mode}, waiting for {mode} mode...")
                time.sleep(1)

    def takeoff(self, target_altitude: int):
        # Arming motors
        print("Arming...")
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm
            0, 0, 0, 0, 0, 0
        )

        # Awaiting arming confirmation
        print("Awaiting confirmation...")
        self.conn.motors_armed_wait()

        # Send takeoff command
        print(f"Armed! Taking off to {target_altitude}m...")
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,          # confirmation
            0, 0, 0, 0, # Params 1-4 (unused)
            0, 0,       # Params 5-6 (Lat/Lon, 0 uses current)
            target_altitude) # Param 7: Target Altitude
        
        # Awaiting takeoff confirmation
        while True:
            msg = self.conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            alt = msg.relative_alt / 1000.0
            print(f"Current altitude: {alt:.2f}", end='\r')
            if alt >= target_altitude * 0.95:  # Allowing a small margin
                print(f"Reached target altitude of {target_altitude}m!")
                break

    def land(self):
        print("Initiating landing...")
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,          # confirmation
            0, 0, 0, 0, # Params 1-4
            0, 0, 0)    # Params 5-7 (Lat/Lon/Alt, 0 uses current)
        
        # Awaiting landing confirmation
        while True:
            msg = self.conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            alt = msg.relative_alt / 1000.0
            print(f"Current altitude: {alt:.2f}", end='\r')
            if alt <= 0.01:
                print("Landed successfully!")
                break

    def goto_position(self, forward=0, right=0, up=0):
        down = -up
        self.conn.mav.set_position_target_local_ned_send(
            0,     # sender's system time (in milliseconds) since boot
            self.conn.target_system,     # target_system
            self.conn.target_component,  # target_component
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # coordinate_frame
            int(0b110111111000),              # bitmask to indicate which dimensions should be ignored
            forward,     # x position in meters (positive is forward/North)
            right,      # y position in meters (positive is right/East)
            down,    # z position in meters (positive is down)
            0,      # x velocity in meters/second
            0,      # y velocity in meters/second
            0,      # z velocity in meters/second
            0,      # x acceleration in meters/second^2
            0,      # y acceleration in meters/second^2
            0,      # z acceleration in meters/second^2
            0,      # yaw in radians
            0       # yaw_rate in radians/second
        )

        print("Command sent. Monitoring movement...")
        while True:
            # Blocking=True with a timeout allows the loop to breathe
            msg = self.conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if msg:
                print(f"Current position: {msg.x:.2f}m, {msg.y:.2f}m, {msg.z:.2f}m", end="")
                '''
                # NED coordinates: x=North, y=East, z=Down
                # We multiply z by -1 to show positive height for humans
                print(f"Current Pos -> N: {msg.x:.2f}m, E: {msg.y:.2f}m, Alt: {-msg.z:.2f}m")
                
                # Check if we are close to the target (10, 0, -10)
                if abs(msg.x - 10) < 0.5 and abs(msg.y - 0) < 0.5 and abs(msg.z - (-10)) < 0.5:
                    print("Target reached!")
                    break
                '''
            else:
                print("Waiting for position update...")
