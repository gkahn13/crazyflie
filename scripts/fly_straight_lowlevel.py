import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/250K'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # take off
        cf.commander.send_hover_setpoint(0, 0, 0, 0.3)
        time.sleep(2.0)

        # -25 for full speed
        cf.commander.send_zdistance_setpoint(0, -20, 0, 0.3) # r p y height
        time.sleep(2.0)

        # land
        # cf.commander.send_hover_setpoint(0, 0, 0, 0.1)
        # time.sleep(1.0)
        cf.commander.send_stop_setpoint()
