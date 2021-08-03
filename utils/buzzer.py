from pymavlink import mavutil
import time
import os

os.environ["MAVLINK20"] = "1"

class Buzzy(object):
    def __init__(self,vehicle):
        self.vehicle = vehicle
        self.TUNE = b""
        # TUNES
        self.TUNE_c_scale = b'MFT200L16<cdefgab>cdefgab>c'
        self.TUNE_alert = b'MFT200L4<AAAA>'
        self.TUNE_london_bridge = b'MFT200L4<GAGFEFG>DEFEFGGAGFEFGDGEC'
        self.TUNE_bad_romance = b'MFT200L4<CDECFEFEDBCDEEEEDCAAEEFEAAEEFEAAEEFECCACACGAGAAACAGAGAAGAGAACAGA>'

    def changeTune(self,newtune):
        self.TUNE = newtune
        return self.TUNE

    def ring(self, itter : int = 1, delay : float = 0.0, start_delay : float = 1.0, T : int = 200 , L : int = 4):
        time.sleep(start_delay)
        for i in range(itter):
            msg = mavutil.mavlink.MAVLink_play_tune_message(0, 0, self.TUNE, tune2=b'')
            self.vehicle.send_mavlink(msg)
            time.sleep(delay)

        return self.TUNE

