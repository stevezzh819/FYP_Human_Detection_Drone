# This python script just toggles firmware parameters (app.start = 1 or 0) over a radio link,
# and the onboard firmware takes care of everything else — stabilization, wall-following, and landing.

import time
import cflib.crtp  # handles the radio link layer (sending and receiving CRTP packets)
from cflib.crazyflie import Crazyflie   # represents the low-level drone object, providing access to all subsystems (logging, parameters, commander, etc.
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# --- Configuration ---
# URI = 'radio://0/80/2M/E7E7E7E7E7'
# URI = 'radio://0/30/2M/E7E7E7E7E7'  # tells the library where and how to connect; radio://<radio_channel>/<datarate>/<radio_address>
URI = 'radio://0/60/2M'
# URI = 'radio://0/120/2M'

# --- Main Control Logic ---
if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    print(f"Connecting to {URI}")

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        
        print("Connection established. Crazyflie is ready.")
        
        # --- 1. START COMMAND ---
        input("Press ENTER to START autonomous flight...")
        
        # Set the 'app.start' parameter to '1' (True)
        cf.param.set_value('app.active', '1')
        print("START command sent. Crazyflie should be taking off and starting its mission.")
        
        # --- 2. FLYING LOOP ---
        # This loop keeps the script running while the Crazyflie is flying
        print("\n*** FLIGHT IN PROGRESS ***")
        print("Press ENTER again to STOP autonomous flight...")
        
        # Wait for user input to stop
        input()
        
        # --- 3. STOP COMMAND ---
        # Set the 'app.start' parameter back to '0' (False)
        cf.param.set_value('app.active', '0')
        print("STOP command sent. Crazyflie should be landing now.")
        
        # Wait a few seconds for the landing to complete
        time.sleep(5)
        
        print("Mission complete. Script exiting.")


# # Run this to see what URI to use:
# import cflib.crtp

# cflib.crtp.init_drivers()

# print("Scanning interfaces for Crazyflies...")
# available = cflib.crtp.scan_interfaces()

# for uri, _ in available:
#     print(uri)