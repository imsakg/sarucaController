from dronekit import Vehicle, VehicleMode, connect, LocationGlobalRelative, LocationGlobal, mavutil,mavlink
from dronekit.mavlink import MAVConnection
from pymavlink.quaternion import QuaternionBase
import math
import time

class Pilot(object):
    def __init__(self, connection_string : str = "/dev/ttyACM0"):
        self.connection_string = connection_string
        self.vehicle = Vehicle
        self.channels = {"1":1500,"2":1500,"3":1500,"4":1500,"5":1500,"6":1500,"7":1500,"8":1500}
        
        self.subModes = {
            0: 'STABILIZE',
            1: 'ACRO',
            2: 'ALT_HOLD',
            3: 'AUTO',
            4: 'GUIDED',
            7: 'CIRCLE',
            9: 'SURFACE',
            16: 'POSHOLD',
            19: 'MANUAL',
            }
        self._initialize=True
        self.wait_ready=None
        self.timeout=30
        self.still_waiting_interval=1
        self.status_printer=None
        self.vehicle_class=None
        self.rate=4
        self.baud=115200
        self.heartbeat_timeout=60
        self.source_system=255
        self.source_component=0
        self.use_native=False
    def setForSim(self):
        self.connection_string = "/dev/ttyACM0"
        self.vehicle = connect(self.connection_string, wait_ready=True)
        

    def launch(self,deafultMode="MANUAL",udp=False, baudRate=115200, port=14550):
        print('Connecting to vehicle on:\t', self.connection_string)
        #self.vehicle = connect(self.connection_string, wait_ready=True)
        self.handler = MAVConnection(self.connection_string, baud=self.baud, source_system=self.source_system, source_component=self.source_component, use_native=self.use_native)
        vehicle = Vehicle(handler=self.handler)
        vehicle.initialize(rate=self.rate, heartbeat_timeout=self.heartbeat_timeout)
        vehicle.wait_ready(still_waiting_interval=self.still_waiting_interval,timeout=self.timeout)
        self.vehicle = vehicle
        self.boot_time = time.time()
        self.master = self.handler.master
        
    def arm(self):
        print("Basic pre-arm checks")
        
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:      
            print(" Waiting for arming...")
            time.sleep(1)
    
    def armForce(self):
        self.vehicle.arm()
    
    def set_home(self, location):
        """
        Set home location.
        """
        self.vehicle.home_location = location

    def disarm(self):
        """
        Disarm vehicle.
        """
        self.vehicle.armed = False
        self.vehicle.flush()
        print("Disarmed")
    
    def changeMode(self, mode= VehicleMode("MANUAL")):
        self.vehicle.mode = mode
        
    
    def getMode(self):
        return self.vehicle.mode

    def takedown(self, depth):
        if depth is not None:
            altitude = float(depth)
            if math.isnan(altitude) or math.isinf(altitude):
                raise ValueError("Altitude was NaN or Infinity. Please provide a real number")
            self.master.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                               0, 0, 0, 0, 0, 0, 0, altitude)
                                               
    def takeoff(self, aTargetAltitude):
        self.vehicle.simple_takeoff(aTargetAltitude) 
        while True:
                    print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)      
                    if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
                        print("Reached target altitude")
                        break
                    time.sleep(1)

    def reboot(self, duration : float = 1.0):
        time.sleep(duration)
        self.vehicle.reboot()
    
    def playMusic(self, sheets : str = "AAAA"):
        #? Todo: Play music
        pass
    
    

    def control(self,id, pwm=0):
        if id < 1:
            print("Channel 1 ve 9 araliginda olmalidir.")
            return

        if id < 9:
            rc_channel_values = [65535 for _ in range(8)]
            rc_channel_values[id - 1] = pwm #self.pwMaper(pwm)
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,             
                *rc_channel_values)
    def pwMaper(self, value):
        speedMin = -1 
        speedMax = 1 
        pwmMin = 1000 
        pwmMax = 2000 
        speedSpan = speedMax - speedMin
        pwmSpan = pwmMax - pwmMin
        valueScaled = float(value - speedMin) / float(speedSpan)
        return pwmMin + (valueScaled * pwmSpan)

    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

            
        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:      
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)      
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)

    """
    #TODO Listen Heartbeat of flightController    
    @vehicle.on_attribute('attitude')
    def attitude_listener(self, name, msg):
        print '%s attribute is: %s' % (name, msg)
    
    """

    """
    Convenience functions for sending immediate/guided mode commands to control the Copter.

    The set of commands demonstrated here include:
    * MAV_CMD_CONDITION_YAW - set direction of the front of the Copter (latitude, longitude)
    * MAV_CMD_DO_SET_ROI - set direction where the camera gimbal is aimed (latitude, longitude, altitude)
    * MAV_CMD_DO_CHANGE_SPEED - set target speed in metres/second.


    The full set of available commands are listed here:
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/
    """

    def condition_yaw(self,heading, relative=True):
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

        This method sets an absolute heading by default, but you can set the `relative` parameter
        to `True` to set yaw relative to the current yaw heading.

        By default the yaw of the vehicle will follow the direction of travel. After setting 
        the yaw using this function there is no way to return to the default yaw "follow direction 
        of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

        For more information see: 
        http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
        """
        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)


    def set_roi(self, location):
        """
        Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a 
        specified region of interest (LocationGlobal).
        The vehicle may also turn to face the ROI.

        For more information see: 
        http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
        """
        # create the MAV_CMD_DO_SET_ROI command
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
            0, #confirmation
            0, 0, 0, 0, #params 1-4
            location.lat,
            location.lon,
            location.alt
            )
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    """
    Functions to make it easy to convert between the different frames-of-reference. In particular these
    make it easy to navigate in terms of "metres from the current position" when using commands that take 
    absolute positions in decimal degrees.

    The methods are approximations only, and may be less accurate over longer distances, and when close 
    to the Earth's poles.get_location_metres

    Specifically, it provides:
    * get_location_metres - Get LocationGlobal (decimal degrees) at distance (m) North & East of a given LocationGlobal.
    * get_distance_metres - Get the distance between two LocationGlobal objects in metres
    * get_bearing - Get the bearing in degrees to a LocationGlobal
    """

    def get_location_metres(self,original_location, dNorth, dEast, alt = None):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
        specified `original_location`. The returned LocationGlobal has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to 
        the current vehicle position.

        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        if alt is not None:
            if type(original_location) is LocationGlobal:
                
                targetlocation=LocationGlobal(newlat, newlon,alt)
            elif type(original_location) is LocationGlobalRelative:
                targetlocation=LocationGlobalRelative(newlat, newlon,alt)
            else:
                raise Exception("Invalid Location object passed")
                
        else:
            if type(original_location) is LocationGlobal:
                
                targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
            elif type(original_location) is LocationGlobalRelative:
                targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
            else:
                raise Exception("Invalid Location object passed")

        return targetlocation;

    def get_distance_metres(self, aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    def get_bearing(self, aLocation1, aLocation2):
        """
        Returns the bearing between the two LocationGlobal objects passed as parameters.

        This method is an approximation, and may not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """	
        off_x = aLocation2.lon - aLocation1.lon
        off_y = aLocation2.lat - aLocation1.lat
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing;

    """
    Functions to move the vehicle to a specified position (as opposed to controlling movement by setting velocity components).

    The methods include:
    * goto_position_target_global_int - Sets position using SET_POSITION_TARGET_GLOBAL_INT command in 
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT frame
    * goto_position_target_local_ned - Sets position using SET_POSITION_TARGET_LOCAL_NED command in 
        MAV_FRAME_BODY_NED frame
    * goto - A convenience function that can use Vehicle.simple_goto (default) or 
        goto_position_target_global_int to travel to a specific position in metres 
        North and East from the current location. 
        This method reports distance to the destination.
    """

    #! broken func
    def goto_position_target_global_int(self, aLocation):
        """
        Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

        For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111111000, # type_mask (only speeds enabled)
            aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
            aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
            aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
            0, # X velocity in NED frame in m/s
            0, # Y velocity in NED frame in m/s
            0, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def goto_position_target_local_ned(self, north, east, down):
        """	
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
        location in the North, East, Down frame.

        It is important to remember that in this frame, positive altitudes are entered as negative 
        "Down" values. So if down is "10", this will be 10 metres below the home altitude.

        Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
        ignored. For more information see: 
        http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.

        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111111000, # type_mask (only positions enabled)
            north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            0, 0, 0, # x, y, z velocity in m/s  (not used)
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def goto(self, dNorth, dEast, alt = None, gotoFunction=Vehicle.simple_goto):
        """
        Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

        The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
        the target position. This allows it to be called with different position-setting commands. 
        By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

        The method reports the distance to target every two seconds.
        """
        
        currentLocation = self.vehicle.location.global_relative_frame
        targetLocation = self.get_location_metres(currentLocation, dNorth, dEast, alt)
        targetDistance = self.get_distance_metres(currentLocation, targetLocation)
        gotoFunction(self.vehicle, targetLocation)
        
        #print "DEBUG: targetLocation: %s" % targetLocation
        #print "DEBUG: targetLocation: %s" % targetDistance

        ''' #! calculate total distance to travel
        while self.vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
            #print "DEBUG: mode: %s" % vehicle.mode.name
            remainingDistance=self.get_distance_metres(self.vehicle.location.global_relative_frame, targetLocation)
            print("Distance to target: ", remainingDistance)
            if remainingDistance<=targetDistance*0.1: #Just below target, in case of undershoot.
                print("Reached target")
                break;
            time.sleep(2)
        '''
        return currentLocation,targetLocation, targetDistance

    """
    Functions that move the vehicle by specifying the velocity components in each direction.
    The two functions use different MAVLink commands. The main difference is
    that depending on the frame used, the NED velocity can be relative to the vehicle
    orientation.

    The methods include:
    * send_ned_velocity - Sets velocity components using SET_POSITION_TARGET_LOCAL_NED command
    * send_global_velocity - Sets velocity components using SET_POSITION_TARGET_GLOBAL_INT command
    """

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors and
        for the specified duration.

        This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
        velocity components 
        (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
        
        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version 
        (sending the message multiple times does not cause problems).
        
        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

        # send command to vehicle on 1 Hz cycle
        for x in range(0,duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

    def send_global_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors.

        This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
        velocity components 
        (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
        
        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version 
        (sending the message multiple times does not cause problems).
        
        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.
        """

        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, # lat_int - X Position in WGS84 frame in 1e7 * meters
            0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x, # X velocity in NED frame in m/s
            velocity_y, # Y velocity in NED frame in m/s
            velocity_z, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

        # send command to vehicle on 1 Hz cycle
        for x in range(0,duration):
            self.vehicle.send_mavlink(msg)
            print(self.vehicle.channels)
            time.sleep(1)
            
    def set_target_depth(self, depth):
        """ Sets the target depth while in depth-hold mode.

        Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

        'depth' is technically an altitude, so set as negative meters below the surface
            -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

        """

        self.master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - self.boot_time)), # ms since boot
            self.master.target_system, self.master.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask=0xdfe,  # ignore everything except z position
            lat_int=0, long_int=0, alt=depth, # (x, y WGS84 frame pos - not used), z [m]
            vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
            afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
            # accelerations in NED frame [N], yaw, yaw_rate
            #  (all not supported yet, ignored in GCS Mavlink)
        ) 

    def set_target_attitude(self, roll, pitch, yaw):
        """ Sets the target attitude while in depth-hold mode.
        'roll', 'pitch', and 'yaw' are angles in degrees.
        """
        # https://mavlink.io/en/messages/common.html#ATTITUDE_TARGET_TYPEMASK
        # 1<<6 = THROTTLE_IGNORE -> allow throttle to be controlled by depth_hold mode
        bitmask = 1<<6

        self.master.mav.set_attitude_target_send(
            int(1e3 * (time.time() - self.boot_time)), # ms since boot
            self.master.target_system, self.master.target_component,
            bitmask,
            # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
            QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
            0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
        )


    def channelOverRide(self, channel, value, overwrite=True):
        """
        Override RC channel with specified value.
        """

        if overwrite:
            channels = self.channels
            channels[str(channel)] = value
        else:
            self.channels = {"1":1500,"2":1500,"3":1500,"4":1500,"5":1500,"6":1500,"7":1500,"8":1500}
            channels = self.channels
            channels[str(channel)] = value

        self.vehicle.channels.overrides = channels
        self.vehicle.flush()
    
    def channelClear(self):
        """
        Clear override on all channels.
        """
        self.channels = {"1":1500,"2":1500,"3":1500,"4":1500,"5":1500,"6":1500,"7":1500,"8":1500}
        self.vehicle.channels.overrides = self.channels
        self.vehicle.flush()
    
    def readRC(self):
        """
        Read RC values.
        """
        return self.vehicle.channels

    # ArduPilot get pressure
    def getPressure(self):
        """
        Get pressure from pressure sensor.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['press_abs']

    # ArduPilot get temperature
    def getTemperature(self):
        """
        Get temperature from temperature sensor.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['temp']

    # ArduPilot get altitude
    def getAltitude(self):
        """
        Get altitude from pressure sensor.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['alt']
    
    # ArduPilot get roll
    def getRoll(self):
        """
        Get roll from IMU.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['roll']
    
    # ArduPilot get pitch
    def getPitch(self):
        """
        Get pitch from IMU.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['pitch']
    
    # ArduPilot get yaw
    def getYaw(self):
        """
        Get yaw from IMU.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['yaw']
    
    # ArduPilot get gps
    def getGPS(self):
        """
        Get gps from IMU.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['gps']
    
    # ArduPilot get gps_heading
    def getGPSHeading(self):
        """
        Get gps heading from IMU.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['gps_heading']
    
    # ArduPilot get gps_alt
    def getGPSAlt(self):
        """
        Get gps altitude from IMU.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['gps_alt']

    # ArduPilot get gps_relative_alt
    def getGPSRelAlt(self):
        """
        Get gps relative altitude from IMU.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['gps_relative_alt']
    
    # ArduPilot get gps_ground_speed
    def getGPSGroundSpeed(self):
        """
        Get gps ground speed from IMU.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['gps_ground_speed']

    # ArduPilot get gps_heading_true
    def getGPSHeadingTrue(self):
        """
        Get gps heading from IMU.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['gps_heading_true']
    
    # ArduPilot get gps_hdop
    def getGPSHDOP(self):
        """
        Get gps hdop from IMU.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['gps_hdop']
    
    # ArduPilot get gps_fix_type
    def getGPSFixType(self):
        """
        Get gps fix type from IMU.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['gps_fix_type']
    
    # ArduPilot get gps_num_sat
    def getGPSNumSat(self):
        """
        Get gps number of satellites from IMU.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['gps_num_sat']
    
    # ArduPilot get gps_lat
    def getGPSLat(self):
        """
        Get gps latitude from IMU.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['gps_lat']
    
    # ArduPilot get gps_lon
    def getGPSLon(self):
        """
        Get gps longitude from IMU.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['gps_lon']
    
    # ArduPilot get gps_alt_ellipsoid
    def getGPSAltEllipsoid(self):
        """
        Get gps altitude ellipsoid from IMU.
        """
        return self.vehicle.parameters['SYSID_THISMAV']['gps_alt_ellipsoid']
    
