import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import numpy as np

URI = 'radio://0/80/250K'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    height = 0.6

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # take off
        cf.commander.send_hover_setpoint(0, 0, 0, height)
        time.sleep(2.0)

        # -25 for full speed
        start_time = time.time()
        while time.time() - start_time < 5.0:
            # cf.commander.send_zdistance_setpoint(0, -20, 0, 0.3) # r p y height
            cf.commander.send_hover_setpoint(1.0, 0., 0, height) # vx vy yawrate height
            time.sleep(0.1)

        # land
        cf.commander.send_hover_setpoint(0, 0, 0, 0.1)
        time.sleep(1.0)
        cf.commander.send_stop_setpoint()
