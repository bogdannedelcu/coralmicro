import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
cflib.crtp.init_drivers()
URI = "udp://127.0.0.1:19850"
with SyncCrazyflie(URI, cf=Crazyflie(rw_cache=None)) as sync:
    cf = sync.cf
    # Wait for log link.
    time.sleep(2)
    with MotionCommander(sync, default_height=1.0) as mc:
        print("[takeoff] hovering")
        time.sleep(15)
        print("[takeoff] landing")
    print("[takeoff] done")
