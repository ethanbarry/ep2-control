from drone_controller import drone_controller
import time

def main():
    # Initialize connection to drone
    drone = drone_controller('udp:127.0.0.1:15551')

    # Set mode to GUIDED
    drone.set_mode('GUIDED')

    # Takeoff to 10 meters
    drone.takeoff(10)

    # Hover for 5 seconds
    print("Hovering for 5 seconds...")
    time.sleep(5)

    # Move to position (25m North, 0m East, 0m Down)
    drone.move(forward=30)
    print("Moving to position (25m North, 0m East, 0m Down)...")

    # Hover for 5 seconds
    print("Hovering for 5 seconds...")
    time.sleep(5)

    # Land drone
    drone.land()

    # Close connection
    print("Mission complete. Closing connection.")

if __name__ == "__main__":
    main()
