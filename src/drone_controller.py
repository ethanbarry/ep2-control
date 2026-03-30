from pymavlink import mavutil
import time
import math

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
        time.sleep(2)

    def land(self, timeout=30):
        print("Initiating landing sequence...")
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )

        start_time = time.time()
        
        while True:
            # 1. Check the Autopilot's internal Landed State
            # land_state 1 = On Ground, 2 = Takeoff, 3 = Landing, 4 = Hovering
            msg_state = self.conn.recv_match(type='EXTENDED_SYS_STATE', blocking=True, timeout=1)
            
            # 2. Check Vertical Velocity (vz) from LOCAL_POSITION_NED
            msg_pos = self.conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            
            print('We arrived at this statement...')

            if msg_state and msg_pos:
                v_z = msg_pos.vz  # Downward velocity in m/s
                landed_flag = msg_state.landed_state
                
                print(f" Descent Rate: {v_z:.2f}m/s | State: {landed_flag}", end='\r')

                # EXIT CONDITION: Autopilot reports "On Ground" (1) 
                # AND vertical speed is near zero
                if landed_flag == 1 and abs(v_z) < 0.1:
                    print("\nTouchdown confirmed by Autopilot.")
                    break

            print('Going past this statement...')

            # 3. Safety Timeout (prevents hanging if landing fails)
            if time.time() - start_time > timeout:
                print("\nLanding timed out. Manual intervention required!")
                break

            time.sleep(0.2)
        time.sleep(2)

    def move(self, forward, right=0, up=0, tolerance=1):
        down = -up
        
        # 1. Get starting position AND current Yaw
        # We need ATTITUDE for the rotation math
        msg_att = self.conn.recv_match(type='ATTITUDE', blocking=True)
        msg_pos = self.conn.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        
        current_yaw = msg_att.yaw  # This is in radians
        start_x, start_y, start_z = msg_pos.x, msg_pos.y, msg_pos.z

        # 2. ROTATION MATH: Convert Body offsets to Local (NED) offsets
        # x_offset (North) = forward*cos(yaw) - right*sin(yaw)
        # y_offset (East)  = forward*sin(yaw) + right*cos(yaw)
        target_x = start_x + (forward * math.cos(current_yaw) - right * math.sin(current_yaw))
        target_y = start_y + (forward * math.sin(current_yaw) + right * math.cos(current_yaw))
        target_z = start_z + down

        # 3. Send the Command (Still using BODY_NED so ArduPilot handles the flight)
        self.conn.mav.set_position_target_local_ned_send(
            0, self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            int(0b110111111000),
            forward, right, down,
            0, 0, 0, 0, 0, 0, 0, 0
        )

        # 4. Tracking Loop (Now target_x/y correctly match the drone's heading)
        print(f"Moving relative to heading ({math.degrees(current_yaw):.1f}°)...")
        total_dist = math.sqrt(forward**2 + right**2 + up**2)
        
        while True:
            msg = self.conn.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            
            distance_to_target = math.sqrt(
                (target_x - msg.x)**2 + 
                (target_y - msg.y)**2 + 
                (target_z - msg.z)**2
            )
            
            progress = max(0, min(100, (1 - (distance_to_target / total_dist)) * 100))
            print(f" Progress: {progress:.1f}% | Dist: {distance_to_target:.2f}m", end='\r')
            
            if distance_to_target < tolerance:
                print(f"\nTarget reached within {tolerance}m!")
                break
            time.sleep(0.1)
        time.sleep(2)

    def rotate(self, angle, speed=10, direction=1, relative=1):
        """
        Rotates the drone to a specific yaw angle.
        :param angle: Angle in degrees
        :param speed: Speed of rotation in deg/s
        :param direction: 1 for CW, -1 for CCW
        :param relative: 1 for relative to current, 0 for absolute
        """
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, # Command ID: 115
            0,                                     # Confirmation
            angle,                                 # Param 1: Target Angle (deg)
            speed,                                 # Param 2: Speed (deg/s)
            direction,                             # Param 3: Direction (1=CW, -1=CCW)
            relative,                              # Param 4: Relative/Absolute
            0, 0, 0                                # Param 5-7: Unused
        )
        print(f"Sent command to rotate {angle} degrees (Relative: {relative})")

        # 1. Capture the starting absolute yaw to determine the goal
        # Note: msg.yaw is in radians
        msg = self.conn.recv_match(type='ATTITUDE', blocking=True)
        start_yaw_deg = math.degrees(msg.yaw)

        # 2. Define the absolute target goal
        if relative:
            # If clockwise (1), add; if counter-clockwise (-1), subtract
            target_yaw_deg = (start_yaw_deg + (angle * direction)) % 360
        else:
            target_yaw_deg = angle % 360

        print(f"Goal: {target_yaw_deg:.1f}° | Starting at: {start_yaw_deg:.1f}°")

        # 3. Blocking wait and progress tracker
        while True:
            msg = self.conn.recv_match(type='ATTITUDE', blocking=True)
            current_yaw_deg = math.degrees(msg.yaw)
            
            # Calculate the shortest distance between two angles
            # We use (target - current + 180) % 360 - 180 to handle the wrap-around at 360/0
            error = (target_yaw_deg - current_yaw_deg + 180) % 360 - 180
            
            print(f" Current Yaw: {current_yaw_deg:.1f}° | Error: {abs(error):.1f}°", end='\r')
            
            # Exit condition: within 1 degree of target
            if abs(error) < 1.0:
                print(f"\nRotation Complete. Final Yaw: {current_yaw_deg:.1f}°")
                break
        time.sleep(2)