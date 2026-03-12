from pymavlink import mavutil

"""
This is the source code for the GNC (guidance, navigation and control) module of this operation.
This program is responsible for operating the drone and locating the target based on the information
provided by the DSP (digital signal processing) module.

This module will interface with two primary component:
 1. The flight controller, using serial communication
 2. Then DSP module, using inter-process communication
"""


def main():
    # Connection Info:
    """
    We open a UDP port and listen on it to receive information from SitL
    This port is chosen because it is the default port used by the simulator
    """
    conn_str = 'udpin:localhost:14540'
    conn = mavutil.mavlink_connection(conn_str)

    conn.wait_heartbeat()

    # (1) The flight controller will receive 
    print("Heartbeat received from (system %u component %u)" % (conn.target_system, conn.target_component))

if __name__ == "__main__":
    main()


"""

"""