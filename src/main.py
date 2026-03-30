from drone_controller import drone_controller
import time

def main():
    # Initialize connection to drone
    drone = drone_controller('udp:127.0.0.1:15551')

    # Takeoff to 10 meters
    drone.set_mode('GUIDED')
    drone.takeoff(10)

    '''
    # We're square dancing with this one, so let's do a little twirl
    for i in range(4):
        drone.move(forward=30)
        drone.rotate(angle=90, relative=True)
    '''
    
    # Land drone
    drone.land()
    print("Mission complete. Closing connection.")

if __name__ == "__main__":
    main()
