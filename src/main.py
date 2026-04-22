from drone_controller import drone_controller
import time

def main():
    # Initialize connection to drone
    time.sleep(30)
    #drone = drone_controller('udp:127.0.0.1:15551')
    drone = drone_controller('/dev/ttyACM0', baud=115200)

    # Takeoff to 10 meters
    drone.set_mode('GUIDED')
    drone.takeoff(2)

    time.sleep(5)
    for i in range(12):
        drone.rotate(angle=30, relative=True)
        time.sleep(2)
    '''
    # We're square dancing with this one, so let's do a little twirl
    for i in range(4):
        drone.move(forward=30)
        drone.rotate(angle=90, relative=True)
    '''
    time.sleep(5)
    
    # Land drone
    drone.land()
    print("Mission complete. Closing connection.")

if __name__ == "__main__":
    main()
