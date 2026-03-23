from drone_controller import drone_controller
import time

def main():
    # Initialize connection to drone
    drone = drone_controller('udp:127.0.0.1:15551')

    # Set mode to GUIDED
    drone.set_mode('GUIDED')

    # Takeoff to 10 meters
    drone.takeoff(10)

    time.sleep(20)

    # Land drone
    drone.land()

    time.sleep(20)

if __name__ == "__main__":
    main()
