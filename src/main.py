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

