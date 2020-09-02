# DroneAPI module
"""
This is the API Reference for the DroneKit-Python API.

The main API is the :py:class:`Vehicle` class.
The code snippet below shows how to use :py:func:`connect` to obtain an instance of a connected vehicle:

.. code:: python

    from dronekit import connect

    # Connect to the Vehicle using "connection string" (in this case an address on network)
    vehicle = connect('127.0.0.1:14550', wait_ready=True)

:py:class:`Vehicle` provides access to vehicle *state* through python attributes
(e.g. :py:attr:`Vehicle.mode`)
and to settings/parameters though the :py:attr:`Vehicle.parameters` attribute.
Asynchronous notification on vehicle attribute changes is available by registering listeners/observers.

Vehicle movement is primarily controlled using the :py:attr:`Vehicle.armed` attribute and
:py:func:`Vehicle.simple_takeoff` and :py:attr:`Vehicle.simple_goto` in GUIDED mode.

Velocity-based movement and control over other vehicle features can be achieved using custom MAVLink messages
(:py:func:`Vehicle.send_mavlink`, :py:func:`Vehicle.message_factory`).

It is also possible to work with vehicle "missions" using the :py:attr:`Vehicle.commands` attribute, and run them in AUTO mode.

All the logging is handled through the builtin Python `logging` module.

A number of other useful classes and methods are listed below.

----
"""

import collections
import copy
import logging
import math
import struct
import time
from enum import Enum

import monotonic
from past.builtins import basestring

from pymavlink import mavutil, mavwp
from pymavlink.dialects.v10 import ardupilotmega, swoop

from dronekit.util import ErrprinterHandler


DetailLookup = {}


#[lidarIntensity', 'hoverBatteryIntensity', 'forwardBatteryIntensity', 'altitudeIntensity', 'windIntensity', 'hoverAttitudeIntensity', 'landingIntensity', 'aerodynamicIntensity', 'airspeedIntensity', 'servoIntensity', 'hoverAssistDetail', 'emergencyLandDetail', 'gpsDetail', 'vibrationDetail', 'hoverMotorDetail', 'forwardMotorDetail', 'lidarDetail', 'hoverBatteryDetail', 'forwardBatteryDetail', 'altitudeDetail', 'windDetail', 'hoverAttitudeDetail', 'landingDetail', 'aerodynamicDetail', 'airspeedDetail', 'servoDetail']


FlagLookup = {"hoverAssist":"Hover Assist","emergencyLand":"Emergency Land","gps":"GPS","vibration":"Vibration","hoverMotor":"Hover System","forwardMotor":"Forward Motor","lidar":"Lidar","hoverBattery":"Hover Battery","forwardBattery":"Forward Battery","altitude":"Altitude","wind":"Wind","hoverAttitude":"Hover System","landing":"Landing","aerodynamic":"Drag","airspeed":"Airspeed","servo":"Flight Control","targetSearchFailed":"Visual Target","adsbFlags":"ADS-B Detection","yaw":"Yaw Error","ekf":"EKF","wings":"Wings","tracking":"Tracking Error","precisionLanding":"Precision Landing"}
DetailLookup["armingCheckFlags1"] = ["Airspeed #1 fail","Airspeed #2 fail","Airspeed #3 fail","Airspeed #4 fail","Autopilot software error","Battery Issue","Logging Failure","No SD Card","Transmitter Problem","No mission loaded","Error in mission","Saftey Switch Activated","Barometer Error","Internal Communications Error","Gyro Calibration Error","Gyro Inconsistency","Proximity","Accel Calibration Error","Accel Inconsistency","LIDAR Error","Compass Calibration Error","Compass Offset","Low Autopilot Voltage","Autopilot Error","Motor Emergency Stop","GPS Blending Unhealthy","Autopilot Parameter Error", "Left Wing Not Connected", "Right Wing Not Connected"]
DetailLookup["armingCheckFlagsCommon"] = ["Altitude Error","GPS Poor Position","GPS Error","Gyro Error","Accel Error","Compass Error","High Magnetic Field","Compass Inconsistent","GPS Positions Different","GPS Inconsistent with Vehicle Estimate","ADSB Threat","Dual GPS Yaw Failure"]
DetailLookup["hoverAssist"] = ["Altitude Low","Speed Low","Unusual Attitude"]
DetailLookup["emergencyLand"] = ["Long Hover Assist Activation","Numerous Hover Assist Activations"]
DetailLookup["gps"] = ["#1 Good","#1 Degraded","#1 Significantly Degraded","#1 Failed","#2 Good","#2 Degraded","#2 Significantly Degraded","#2 Failed"]
DetailLookup["hoverMotor"] = ["#1 Failure","#2 Failure","#3 Failure","#4 Failure","Oscillations","Offset","Output Saturation","High Power"]
DetailLookup["forwardMotor"] = ["#1 Failure","#2 Failure","Output Saturation","High Power"]
DetailLookup["lidar"] = ["Terrain Separation","Failure"]
DetailLookup["forwardBattery"] = ["< 10% remaining","Failure Detected"]
DetailLookup["hoverBattery"] = ["< 10% remaining","Failure Detected"]
DetailLookup["altitude"] = ["High","Low"]
DetailLookup["landing"] = ["Hard Landing","Unusual Attitude"]
DetailLookup["aerodynamic"] = ["Assymetric Drag","High Drag","ESC Temp High"]
DetailLookup["airspeed"] = ["Low","High","#1 Fail","#2 Fail","#3 Fail","#4 Fail"]
DetailLookup["servo"] = ["Elevator Offset","Elevator Failure","Aileron Offset","Aileron Failure","Rudder Offset","Rudder Failure"]
DetailLookup["targetSearchFailed"] = ["Landing Aborted", "GPS Landing", "Contingency Divert"]
DetailLookup["adsbFlags"] = ["Aircraft Detected", "Threat Detected", "Avoiding Threat"]
DetailLookup["yaw"] = ["Dual GPS Failure", "Compass Fallback Not Healthy", "Fallback Active", "Compass Not Healthy", "Compass Inconsistent"]
DetailLookup["ekf"] = ["Velocity Unhealthy", "Position Unhealthy", "Altitude Unhealthy", "Yaw Unhealthy", "Terrain Unhealthy", "GPS Timesync Error"]
DetailLookup["wings"] = ["Left Not Connected", "Right Not Connected"]
DetailLookup["tracking"] = ["Flight Geography Left","Flight Geography Right", "Flight Geography High", "Flight Geography Low", "Contingency Volume Left", "Contingency Volume Right", "Contingency Volume High"]
DetailLookup["precisionLanding"] = ["Target Not Found", "Using GPS", "Using Next Waypoint"]
ARMING_CHECK_IRREGULAR_LENGTH = 29
ARMING_CHECK_COMMON_LENGTH    = 12
FLAGS_LENGTH                  = 23
MStoKnotsConversionFactor = 1.94384 
MeterstoFeettConversionFactor = 3.28084

def bit_format(length, value):
    if value.bit_length() > length:
        extra_bits = value.bit_length() - length
        logging.error("%d unknown flags or details were raised. Number of bits incorrect." % extra_bits)
    return ('{:0%db}'%length).format(value)[-length:]  # ensure correct number of bits


class flightStatus(Enum):
    DISSARMED_ON_GROUND = 0
    MOTORS_IDLING = 1
    TAKEOFF = 2
    TRANSITION_TO_FORWARD_FLIGHT = 3
    FORWARD_FLIGHT = 4
    TRANSITION_TO_HOVER = 5
    SEARCHING_FOR_TARGET = 6
    LANDING = 7
    ORBIT = 8
    EMERGENCY_LAND = 9
    PARACHUTE_DEPLOYED = 10
    ABORTING_TAKEOFF = 11
    ABNORMAL = 12
    NONE = 13

class navigationState(Enum):
    STRAIT = 0
    TURN = 1
    ORBIT_STATE = 2

class APIException(Exception):
    """
    Base class for DroneKit related exceptions.

    :param String message: Message string describing the exception
    """


class TimeoutError(APIException):
    '''Raised by operations that have timeouts.'''

class LocationGlobal(object):
    """
    A global location object.

    The latitude and longitude are relative to the `WGS84 coordinate system <http://en.wikipedia.org/wiki/World_Geodetic_System>`_.
    The altitude is relative to mean sea-level (MSL).

    For example, a global location object with altitude 30 metres above sea level might be defined as:

    .. code:: python

       LocationGlobal(-34.364114, 149.166022, 30)

    .. todo:: FIXME: Location class - possibly add a vector3 representation.

    An object of this type is owned by :py:attr:`Vehicle.location`. See that class for information on
    reading and observing location in the global frame.

    :param lat: Latitude.
    :param lon: Longitude.
    :param alt: Altitude in meters relative to mean sea-level (MSL).
    """

    def __init__(self, lat, lon, alt=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt

        # This is for backward compatibility.
        self.local_frame = None
        self.global_frame = None


class SwoopStatus(object):
    """
    SWOOP STATUS.

    An object of this type is returned by :py:attr:`Vehicle.swoopstatus`.
    """
    def __init__(self, swoopstatus):
        self.flightStatus = swoopstatus.flightStatus
        self.navigationState = swoopstatus.navigationState
        self.previousWaypointType = swoopstatus.previousWaypointType
        self.currentWaypointType = swoopstatus.currentWaypointType
        self.nextWaypointType = swoopstatus.nextWaypointType
        self.previousNavWaypointIndex = swoopstatus.previousNavWaypointIndex
        self.currentNavWaypointIndex = swoopstatus.currentNavWaypointIndex
        self.nextNavWaypointIndex = swoopstatus.nextNavWaypointIndex
        self.waypointJumper = swoopstatus.waypointJumper
        self.turnAroundOK = swoopstatus.turnAroundOK
        self.forwardDivertOK = swoopstatus.forwardDivertOK
        
    def __str__(self):
        return "Swoop Status: Status={}, State={}, Previous Waypoint:{} Type:{}, Current Waypoint={} Type={}, Next Nav Waypoint={} Type={}, Jumper={}, Turn Around Ok={}, Forward Diversion Ok={}".format(self.flightStatus, self.navigationState, self.previousNavWaypointIndex, self.previousWaypointType, self.currentNavWaypointIndex, self.currentWaypointType, self.nextNavWaypointIndex, self.nextWaypointType, self.waypointJumper, self.turnAroundOK, self.forwardDivertOK  )

    def statusText(self):
        return flightStatus(self.flightStatus).name
    
    def navStateText(self):
        return navigationState(self.navigationState).name

class Position(object):
    """
    Swoop Position

    An object of this type is returned by :py:attr:`Vehicle.swoopstatus`.
    """

    def __init__(self, lat, lon, altMeters, lidarMeters,track,heading):
        self.lat = lat
        self.lon = lon
        self.alt = round(altMeters * MeterstoFeettConversionFactor)
        self.lidar = round(lidarMeters * MeterstoFeettConversionFactor)
        self.track = track
        self.heading =  heading

    def __str__(self):
        return "Swoop Position: lat={}, lon={}, alt={}, lidar={}, track={}, heading={}".format(self.lat, self.lon,self.alt,self.lidar,self.track,self.heading)

    def alt_mtrs(self):
        return self.alt / MeterstoFeettConversionFactor


class Speed(object):
    """
    SWOOP Position.

    An object of this type is returned by :py:attr:`Vehicle.swoopstatus`.
    """
    def __init__(self, groundspeed, airspeed, TAS, TAS_Set, climb):
        # Converts each to knot
        self.ground = round(groundspeed * MStoKnotsConversionFactor*10)/10
        self.air = round(airspeed * MStoKnotsConversionFactor*10)/10
        self.TAS = round(TAS * MStoKnotsConversionFactor*10)/10
        self.TAS_Set = round(TAS_Set * MStoKnotsConversionFactor*10)/10
        self.climb = climb

    def __str__(self):
        return "Swoop Speed: groundspeed={}, airspeed={}, TAS={}, TAS_Set={}, climb={}".format(self.ground, self.air,self.TAS,self.TAS_Set,self.climb)
    
    def ground_ms(self):
        return self.ground / MStoKnotsConversionFactor

    def air_ms(self):
        return self.air / MStoKnotsConversionFactor

    def TAS_Set_ms(self):
        return self.TAS_Set / MStoKnotsConversionFactor


class WindDetails(object):
    def __init__(self, status, wind, savedWind):
        self.status = status
        self.direction = wind.direction
        self.speed = wind.speed
        self.speed_z = wind.speed_z

        if savedWind is None:
            self.savedDirection = 0
            self.savedSpeed = 0
            self.savedSpeed_z = 0
        else:
            self.savedDirection = wind.direction
            self.savedSpeed = wind.speed
            self.savedSpeed_z = wind.speed_z
    
    def knots(self,d,s,sz):
        return [round(d), round(s* MStoKnotsConversionFactor,1), round(sz* MStoKnotsConversionFactor,1)]

    def ms(self,d,s,sz):
        return [d,s,sz]

    def wind_dict_knots(self,d,s,sz):
        return self.knots(self.direction,self.speed,self.speed_z)

    def wind_bestreference_ms(self):
        if (self.status in ['FORWARD_FLIGHT','EMERGENCY_LAND']):
            return self.ms(self.direction,self.speed,self.speed_z)
        else:
            return self.ms(self.savedDirection,self.savedSpeed,self.savedSpeed_z)

    def __str__(self):
        return "Wind: Direction={}, Speed={}m/s, Speed Z={}m/s".format(self.direction, self.speed, self.speed_z)
    
    

class SensorOffsets(object):
    def __init__(self, temp):
        self.pixhawk_temp = temp
    

class SwoopInFlightFlags(object):
    def __init__(self, inflightFlagsMsg):
        self.swoop_flags =None
        self.swoop_flags_id = bit_format(length=FLAGS_LENGTH, value=0)
        self.autopilotTriggerContingency = False

        flags = {}
        flags["Flags"] = []

        fields = inflightFlagsMsg.get_fieldnames()
        #logging.info(fields)
        self.autopilotTriggerContingency = False

        i = 0
        while i < len(fields):
            if (fields[i] == "maximumIntensity"):
                flags["maxIntensity"] = getattr(inflightFlagsMsg,fields[i])
            elif (fields[i] == "inflightFlags"):
                self.swoop_flags_id = bit_format(length=FLAGS_LENGTH, value=inflightFlagsMsg.inflightFlags) # [int(digit) for digit in '{:016b}'.format(m.inflightFlags)]
            elif (fields[i].endswith("Intensity")):
                #logging.info("Intensity:" + fields[i])
                if getattr(inflightFlagsMsg,fields[i]) > 0:
                    flag = {}
                    flag["Type"] = FlagLookup[fields[i].replace("Intensity", "")]
                    flag["Intensity"] = getattr(inflightFlagsMsg,fields[i])

                    if (fields[i].replace("Intensity", "") != "adsbFlags"):
                        if flag["Intensity"] >= 3:
                            self.autopilotTriggerContingency = True

                    detailValue = getattr(inflightFlagsMsg,fields[i].replace("Intensity", "Detail"))
                    if detailValue > 0:
                        flag["DetailID"] = detailValue
                        flag["Detail"] = self.detail_lookup(fields[i].replace("Intensity", ""),detailValue)

                    flags["Flags"].append(flag)

            i += 1
            self.swoop_flags = flags

    def __str__(self):
        return "Flags: Auto Trigger Contingency={}, SatcomFlags={}, Flags={}".format(self.autopilotTriggerContingency, self.swoop_flags_id,self.swoop_flags)
    
    def detail_lookup(self, lookupIdentifier,value):
        len(DetailLookup[lookupIdentifier])
        lookup = []
        i = len(DetailLookup[lookupIdentifier])

        while i > 0:

            if (value / 2**(i-1)) >= 1:
                value = value - (2**(i-1))
                lookup.append(DetailLookup[lookupIdentifier][i - 1])
            i -= 1

        return lookup


class Battery(object):
    """
    System battery information.
    An object of this type is returned by :py:attr:`Vehicle.battery`.
    :param voltage: Battery voltage in millivolts.
    :param current: Battery current, in 10 * milliamperes. ``None`` if the autopilot does not support current measurement.
    :param level: Remaining battery energy. ``None`` if the autopilot cannot estimate the remaining battery.
    """

    def __init__(self, voltage, current, level):
        self.voltage = voltage / 1000.0
        if current == -1:
            self.current = None
        else:
            self.current = current / 100.0
        if level == -1:
            self.level = None
        else:
            self.level = level

    def __str__(self):
        return "Battery:voltage={},current={},level={}".format(self.voltage, self.current,
                                                               self.level)

class SwoopArmingFlags(object):
    def __init__(self, swooparmingflags):
        self.swoop_arming_check_irregular = bit_format(length=ARMING_CHECK_IRREGULAR_LENGTH, value=0)
        self.swoop_arming_check_common = bit_format(length=ARMING_CHECK_COMMON_LENGTH, value=0)
        self.droneready = False

        droneready = {}
        droneready["common"] = []
        droneready["irregular"] = []

        if (swooparmingflags.armingCheckStatus > 0):
            droneready["ready"] = True
            self.droneready = True
            self.swoop_arming_check_irregular = bit_format(length=ARMING_CHECK_IRREGULAR_LENGTH, value=0)
            self.swoop_arming_check_common = bit_format(length=ARMING_CHECK_COMMON_LENGTH, value=0)
        else:
            droneready["ready"] = False
            self.droneready = False
            self.swoop_arming_check_irregular = bit_format(length=ARMING_CHECK_IRREGULAR_LENGTH, value=swooparmingflags.armingCheckFlags1)
            self.swoop_arming_check_common = bit_format(length=ARMING_CHECK_COMMON_LENGTH, value=swooparmingflags.armingCheckFlagsCommon)

            armingCheckFlags1 = self.detail_lookup('armingCheckFlags1',swooparmingflags.armingCheckFlags1)

            if(len(armingCheckFlags1) > 0):
                droneready["irregular"] = armingCheckFlags1

            armingCheckFlagsCommon = self.detail_lookup('armingCheckFlagsCommon',swooparmingflags.armingCheckFlagsCommon)
            if (len(armingCheckFlagsCommon) > 0):
                droneready["common"] = armingCheckFlagsCommon

        self.swoop_droneready = droneready

    def __str__(self):
        return "Arming Checks: Drone Ready={}, SatcomRegular={}, SatcomIrregular={}, Checks={}".format(self.droneready, self.swoop_arming_check_common, self.swoop_arming_check_irregular,self.swoop_droneready)

    def detail_lookup(self, lookupIdentifier,value):
        len(DetailLookup[lookupIdentifier])
        lookup = []
        i = len(DetailLookup[lookupIdentifier])

        while i > 0:

            if (value / 2**(i-1)) >= 1:
                value = value - (2**(i-1))
                lookup.append(DetailLookup[lookupIdentifier][i - 1])
            i -= 1

        return lookup
    


class Version(object):
    """
    Autopilot version and type.

    An object of this type is returned by :py:attr:`Vehicle.version`.

    The version number can be read in a few different formats. To get it in a human-readable
    format, just print `vehicle.version`.  This might print something like "APM:Copter-3.3.2-rc4".

    .. versionadded:: 2.0.3

    .. py:attribute:: major

        Major version number (integer).

    .. py:attribute::minor

        Minor version number (integer).

    .. py:attribute:: patch

        Patch version number (integer).

    .. py:attribute:: release

        Release type (integer). See the enum `FIRMWARE_VERSION_TYPE <http://mavlink.org/messages/common#http://mavlink.org/messages/common#FIRMWARE_VERSION_TYPE_DEV>`_.

        This is a composite of the product release cycle stage (rc, beta etc) and the version in that cycle - e.g. 23.

    """
    def __init__(self, raw_version, autopilot_type, vehicle_type):
        self.autopilot_type = autopilot_type
        self.vehicle_type = vehicle_type
        self.raw_version = raw_version
        if raw_version is None:
            self.major = None
            self.minor = None
            self.patch = None
            self.release = None
        else:
            self.major   = raw_version >> 24 & 0xFF
            self.minor   = raw_version >> 16 & 0xFF
            self.patch   = raw_version >> 8  & 0xFF
            self.release = raw_version & 0xFF

    def is_stable(self):
        """
        Returns True if the autopilot reports that the current firmware is a stable
        release (not a pre-release or development version).
        """
        return self.release == 255

    def release_version(self):
        """
        Returns the version within the release type (an integer).
        This method returns "23" for Copter-3.3rc23.
        """
        if self.release is None:
            return None
        if self.release == 255:
            return 0
        return self.release % 64

    def release_type(self):
        """
        Returns text describing the release type e.g. "alpha", "stable" etc.
        """
        if self.release is None:
            return None
        types = ["dev", "alpha", "beta", "rc"]
        return types[self.release >> 6]

    def __str__(self):
        prefix = ""

        if self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            prefix += "APM:"
        elif self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            prefix += "PX4"
        else:
            prefix += "UnknownAutoPilot"

        if self.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
            prefix += "Copter-"
        elif self.vehicle_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            prefix += "Plane-"
        elif self.vehicle_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            prefix += "Rover-"
        else:
            prefix += "UnknownVehicleType%d-" % self.vehicle_type

        if self.release_type() is None:
            release_type = "UnknownReleaseType"
        elif self.is_stable():
            release_type = ""
        else:
            # e.g. "-rc23"
            release_type = "-" + str(self.release_type()) + str(self.release_version())

        return prefix + "%s.%s.%s" % (self.major, self.minor, self.patch) + release_type


class Capabilities:
    """
    Autopilot capabilities (supported message types and functionality).

    An object of this type is returned by :py:attr:`Vehicle.capabilities`.

    See the enum
    `MAV_PROTOCOL_CAPABILITY <http://mavlink.org/messages/common#MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT>`_.

    .. versionadded:: 2.0.3


    .. py:attribute:: mission_float

        Autopilot supports MISSION float message type (Boolean).

    .. py:attribute:: param_float

        Autopilot supports the PARAM float message type (Boolean).

    .. py:attribute:: mission_int

        Autopilot supports MISSION_INT scaled integer message type (Boolean).

    .. py:attribute:: command_int

        Autopilot supports COMMAND_INT scaled integer message type (Boolean).

    .. py:attribute:: param_union

        Autopilot supports the PARAM_UNION message type (Boolean).

    .. py:attribute:: ftp

        Autopilot supports ftp for file transfers (Boolean).

    .. py:attribute:: set_attitude_target

        Autopilot supports commanding attitude offboard (Boolean).

    .. py:attribute:: set_attitude_target_local_ned

        Autopilot supports commanding position and velocity targets in local NED frame (Boolean).

    .. py:attribute:: set_altitude_target_global_int

        Autopilot supports commanding position and velocity targets in global scaled integers (Boolean).

    .. py:attribute:: terrain

        Autopilot supports terrain protocol / data handling (Boolean).

    .. py:attribute:: set_actuator_target

        Autopilot supports direct actuator control (Boolean).

    .. py:attribute:: flight_termination

        Autopilot supports the flight termination command (Boolean).

    .. py:attribute:: compass_calibration

        Autopilot supports onboard compass calibration (Boolean).
    """
    def __init__(self, capabilities):
        self.mission_float                  = (((capabilities >> 0)  & 1) == 1)
        self.param_float                    = (((capabilities >> 1)  & 1) == 1)
        self.mission_int                    = (((capabilities >> 2)  & 1) == 1)
        self.command_int                    = (((capabilities >> 3)  & 1) == 1)
        self.param_union                    = (((capabilities >> 4)  & 1) == 1)
        self.ftp                            = (((capabilities >> 5)  & 1) == 1)
        self.set_attitude_target            = (((capabilities >> 6)  & 1) == 1)
        self.set_attitude_target_local_ned  = (((capabilities >> 7)  & 1) == 1)
        self.set_altitude_target_global_int = (((capabilities >> 8)  & 1) == 1)
        self.terrain                        = (((capabilities >> 9)  & 1) == 1)
        self.set_actuator_target            = (((capabilities >> 10) & 1) == 1)
        self.flight_termination             = (((capabilities >> 11) & 1) == 1)
        self.compass_calibration            = (((capabilities >> 12) & 1) == 1)


class VehicleMode(object):
    """
    This object is used to get and set the current "flight mode".

    The flight mode determines the behaviour of the vehicle and what commands it can obey.
    The recommended flight modes for *DroneKit-Python* apps depend on the vehicle type:

    * Copter apps should use ``AUTO`` mode for "normal" waypoint missions and ``GUIDED`` mode otherwise.
    * Plane and Rover apps should use the ``AUTO`` mode in all cases, re-writing the mission commands if "dynamic"
      behaviour is required (they support only a limited subset of commands in ``GUIDED`` mode).
    * Some modes like ``RETURN_TO_LAUNCH`` can be used on all platforms. Care should be taken
      when using manual modes as these may require remote control input from the user.

    The available set of supported flight modes is vehicle-specific (see
    `Copter Modes <http://copter.ardupilot.com/wiki/flying-arducopter/flight-modes/>`_,
    `Plane Modes <http://plane.ardupilot.com/wiki/flying/flight-modes/>`_,
    `Rover Modes <http://rover.ardupilot.com/wiki/configuration-2/#mode_meanings>`_). If an unsupported mode is set the script
    will raise a ``KeyError`` exception.

    The :py:attr:`Vehicle.mode` attribute can be queried for the current mode.
    The code snippet below shows how to observe changes to the mode and then read the value:

    .. code:: python

        #Callback definition for mode observer
        def mode_callback(self, attr_name):
            print "Vehicle Mode", self.mode

        #Add observer callback for attribute `mode`
        vehicle.add_attribute_listener('mode', mode_callback)

    The code snippet below shows how to change the vehicle mode to AUTO:

    .. code:: python

        # Set the vehicle into auto mode
        vehicle.mode = VehicleMode("AUTO")

    For more information on getting/setting/observing the :py:attr:`Vehicle.mode`
    (and other attributes) see the :ref:`attributes guide <vehicle_state_attributes>`.

    .. py:attribute:: name

        The mode name, as a ``string``.
    """

    def __init__(self, name):
        self.name = name

    def __str__(self):
        return "VehicleMode:%s" % self.name

    def __eq__(self, other):
        return self.name == other

    def __ne__(self, other):
        return self.name != other


class SystemStatus(object):
    """
    This object is used to get and set the current "system status".

    An object of this type is returned by :py:attr:`Vehicle.system_status`.

    .. py:attribute:: state

        The system state, as a ``string``.
    """

    def __init__(self, state):
        self.state = state

    def __str__(self):
        return "SystemStatus:%s" % self.state

    def __eq__(self, other):
        return self.state == other

    def __ne__(self, other):
        return self.state != other


class HasObservers(object):
    def __init__(self):
        logging.basicConfig()
        self._logger = logging.getLogger(__name__)

        # A mapping from attr_name to a list of observers
        self._attribute_listeners = {}
        self._attribute_cache = {}

    def add_attribute_listener(self, attr_name, observer):
        """
        Add an attribute listener callback.

        The callback function (``observer``) is invoked differently depending on the *type of attribute*.
        Attributes that represent sensor values or which are used to monitor connection status are updated
        whenever a message is received from the vehicle. Attributes which reflect vehicle "state" are
        only updated when their values change (for example :py:attr:`Vehicle.system_status`,
        :py:attr:`Vehicle.armed`, and :py:attr:`Vehicle.mode`).

        The callback can be removed using :py:func:`remove_attribute_listener`.

        .. note::

            The :py:func:`on_attribute` decorator performs the same operation as this method, but with
            a more elegant syntax. Use ``add_attribute_listener`` by preference if you will need to remove
            the observer.

        The argument list for the callback is ``observer(object, attr_name, attribute_value)``:

        * ``self`` - the associated :py:class:`Vehicle`. This may be compared to a global vehicle handle
          to implement vehicle-specific callback handling (if needed).
        * ``attr_name`` - the attribute name. This can be used to infer which attribute has triggered
          if the same callback is used for watching several attributes.
        * ``value`` - the attribute value (so you don't need to re-query the vehicle object).

        The example below shows how to get callbacks for (global) location changes:

        .. code:: python

            #Callback to print the location in global frame
            def location_callback(self, attr_name, msg):
                print "Location (Global): ", msg

            #Add observer for the vehicle's current location
            vehicle.add_attribute_listener('global_frame', location_callback)

        See :ref:`vehicle_state_observe_attributes` for more information.

        :param String attr_name: The name of the attribute to watch (or '*' to watch all attributes).
        :param observer: The callback to invoke when a change in the attribute is detected.

        """
        listeners_for_attr = self._attribute_listeners.get(attr_name)
        if listeners_for_attr is None:
            listeners_for_attr = []
            self._attribute_listeners[attr_name] = listeners_for_attr
        if observer not in listeners_for_attr:
            listeners_for_attr.append(observer)

    def remove_attribute_listener(self, attr_name, observer):
        """
        Remove an attribute listener (observer) that was previously added using :py:func:`add_attribute_listener`.

        For example, the following line would remove a previously added vehicle 'global_frame'
        observer called ``location_callback``:

        .. code:: python

            vehicle.remove_attribute_listener('global_frame', location_callback)

        See :ref:`vehicle_state_observe_attributes` for more information.

        :param String attr_name: The attribute name that is to have an observer removed (or '*' to remove an 'all attribute' observer).
        :param observer: The callback function to remove.

        """
        listeners_for_attr = self._attribute_listeners.get(attr_name)
        if listeners_for_attr is not None:
            listeners_for_attr.remove(observer)
            if len(listeners_for_attr) == 0:
                del self._attribute_listeners[attr_name]

    def notify_attribute_listeners(self, attr_name, value, cache=False):
        """
        This method is used to update attribute observers when the named attribute is updated.

        You should call it in your message listeners after updating an attribute with
        information from a vehicle message.

        By default the value of ``cache`` is ``False`` and every update from the vehicle is sent to listeners
        (whether or not the attribute has changed).  This is appropriate for attributes which represent sensor
        or heartbeat-type monitoring.

        Set ``cache=True`` to update listeners only when the value actually changes (cache the previous
        attribute value). This should be used where clients will only ever need to know the value when it has
        changed. For example, this setting has been used for notifying :py:attr:`mode` changes.

        See :ref:`example_create_attribute` for more information.

        :param String attr_name: The name of the attribute that has been updated.
        :param value: The current value of the attribute that has been updated.
        :param Boolean cache: Set ``True`` to only notify observers when the attribute value changes.
        """
        # Cached values are not re-sent if they are unchanged.
        if cache:
            if self._attribute_cache.get(attr_name) == value:
                return
            self._attribute_cache[attr_name] = value

        # Notify observers.
        for fn in self._attribute_listeners.get(attr_name, []):
            try:
                fn(self, attr_name, value)
            except Exception:
                self._logger.exception('Exception in attribute handler for %s' % attr_name, exc_info=True)

        for fn in self._attribute_listeners.get('*', []):
            try:
                fn(self, attr_name, value)
            except Exception:
                self._logger.exception('Exception in attribute handler for %s' % attr_name, exc_info=True)

    def on_attribute(self, name):
        """
        Decorator for attribute listeners.

        The decorated function (``observer``) is invoked differently depending on the *type of attribute*.
        Attributes that represent sensor values or which are used to monitor connection status are updated
        whenever a message is received from the vehicle. Attributes which reflect vehicle "state" are
        only updated when their values change (for example :py:func:`Vehicle.system_status`,
        :py:attr:`Vehicle.armed`, and :py:attr:`Vehicle.mode`).

        The argument list for the callback is ``observer(object, attr_name, attribute_value)``

        * ``self`` - the associated :py:class:`Vehicle`. This may be compared to a global vehicle handle
          to implement vehicle-specific callback handling (if needed).
        * ``attr_name`` - the attribute name. This can be used to infer which attribute has triggered
          if the same callback is used for watching several attributes.
        * ``msg`` - the attribute value (so you don't need to re-query the vehicle object).

        .. note::

            There is no way to remove an attribute listener added with this decorator. Use
            :py:func:`add_attribute_listener` if you need to be able to remove
            the :py:func:`attribute listener <remove_attribute_listener>`.

        The code fragment below shows how you can create a listener for the attitude attribute.

        .. code:: python

            @vehicle.on_attribute('attitude')
            def attitude_listener(self, name, msg):
                print '%s attribute is: %s' % (name, msg)

        See :ref:`vehicle_state_observe_attributes` for more information.

        :param String name: The name of the attribute to watch (or '*' to watch all attributes).
        :param observer: The callback to invoke when a change in the attribute is detected.
        """

        def decorator(fn):
            if isinstance(name, list):
                for n in name:
                    self.add_attribute_listener(n, fn)
            else:
                self.add_attribute_listener(name, fn)

        return decorator


class ChannelsOverride(dict):
    """
    A dictionary class for managing Vehicle channel overrides.

    Channels can be read, written, or cleared by index or using a dictionary syntax.
    To clear a value, set it to ``None`` or use ``del`` on the item.

    An object of this type is returned by :py:attr:`Vehicle.channels.overrides <Channels.overrides>`.

    For more information and examples see :ref:`example_channel_overrides`.
    """

    def __init__(self, vehicle):
        self._vehicle = vehicle
        self._count = 8  # Fixed by MAVLink
        self._active = True

    def __getitem__(self, key):
        return dict.__getitem__(self, str(key))

    def __setitem__(self, key, value):
        if not (0 < int(key) <= self._count):
            raise KeyError('Invalid channel index %s' % key)
        if not value:
            try:
                dict.__delitem__(self, str(key))
            except:
                pass
        else:
            dict.__setitem__(self, str(key), value)
        self._send()

    def __delitem__(self, key):
        dict.__delitem__(self, str(key))
        self._send()

    def __len__(self):
        return self._count

    def _send(self):
        if self._active:
            overrides = [0] * 8
            for k, v in self.items():
                overrides[int(k) - 1] = v
            self._vehicle._master.mav.rc_channels_override_send(0, 0, *overrides)


class Locations(HasObservers):
    """
    An object for holding location information in global, global relative and local frames.

    :py:class:`Vehicle` owns an object of this type. See :py:attr:`Vehicle.location` for information on
    reading and observing location in the different frames.

    The different frames are accessed through the members, which are created with this object.
    They can be read, and are observable.
    """

    def __init__(self, vehicle):
        super(Locations, self).__init__()

        self._lat = None
        self._lon = None
        self._alt = None
        self._relative_alt = None

        @vehicle.on_message('GLOBAL_POSITION_INT')
        def listener(vehicle, name, m):
            (self._lat, self._lon) = (m.lat / 1.0e7, m.lon / 1.0e7)
            self._relative_alt = m.relative_alt / 1000.0

            if self._alt is not None or m.alt != 0:
                # Require first alt value to be non-0
                # TODO is this the proper check to do?
                self._alt = m.alt / 1000.0
    @property
    def global_frame(self):
        """
        Location in global frame (a :py:class:`LocationGlobal`).

        The latitude and longitude are relative to the
        `WGS84 coordinate system <http://en.wikipedia.org/wiki/World_Geodetic_System>`_.
        The altitude is relative to mean sea-level (MSL).

        This is accessed through the :py:attr:`Vehicle.location` attribute:

        .. code-block:: python

            print "Global Location: %s" % vehicle.location.global_frame
            print "Sea level altitude is: %s" % vehicle.location.global_frame.alt

        Its ``lat`` and ``lon`` attributes are populated shortly after GPS becomes available.
        The ``alt`` can take several seconds longer to populate (from the barometer).
        Listeners are not notified of changes to this attribute until it has fully populated.

        To watch for changes you can use :py:func:`Vehicle.on_attribute` decorator or
        :py:func:`add_attribute_listener` (decorator approach shown below):

        .. code-block:: python

            @vehicle.on_attribute('location.global_frame')
            def listener(self, attr_name, value):
                print " Global: %s" % value

            #Alternatively, use decorator: ``@vehicle.location.on_attribute('global_frame')``.
        """
        return LocationGlobal(self._lat, self._lon, self._alt)
                

class Vehicle(HasObservers):
    """
    The main vehicle API.

    Vehicle state is exposed through 'attributes' (e.g. :py:attr:`heading`). All attributes can be
    read, and some are also settable
    (:py:attr:`mode`, :py:attr:`armed` and :py:attr:`home_location`).

    Attributes can also be asynchronously monitored for changes by registering listener callback
    functions.

    Vehicle "settings" (parameters) are read/set using the :py:attr:`parameters` attribute.
    Parameters can be iterated and are also individually observable.

    Vehicle movement is primarily controlled using the :py:attr:`armed` attribute and
    :py:func:`simple_takeoff` and :py:func:`simple_goto` in GUIDED mode.

    It is also possible to work with vehicle "missions" using the :py:attr:`commands` attribute,
    and run them in AUTO mode.

    STATUSTEXT log messages from the autopilot are handled through a separate logger.
    It is possible to configure the log level, the formatting, etc. by accessing the logger, e.g.:

    .. code-block:: python

        import logging
        autopilot_logger = logging.getLogger('autopilot')
        autopilot_logger.setLevel(logging.DEBUG)

    The guide contains more detailed information on the different ways you can use
    the ``Vehicle`` class:

    - :doc:`guide/vehicle_state_and_parameters`
    - :doc:`guide/copter/guided_mode`
    - :doc:`guide/auto_mode`


    .. note::

        This class currently exposes just the attributes that are most commonly used by all
        vehicle types. if you need to add additional attributes then subclass ``Vehicle``
        as demonstrated in :doc:`examples/create_attribute`.

        Please then :doc:`contribute <contributing/contributions_api>` your additions back
        to the project!
    """

    def __init__(self, handler):
        super(Vehicle, self).__init__()

        self._logger = logging.getLogger(__name__)  # Logger for DroneKit
        self._autopilot_logger = logging.getLogger('autopilot')  # Logger for the autopilot messages
        # MAVLink-to-logging-module log severity mappings
        self._mavlink_statustext_severity = {
            0: logging.CRITICAL,
            1: logging.CRITICAL,
            2: logging.CRITICAL,
            3: logging.ERROR,
            4: logging.WARNING,
            5: logging.INFO,
            6: logging.INFO,
            7: logging.DEBUG
        }

        self._handler = handler
        self._master = handler.master

        # Cache all updated attributes for wait_ready.
        # By default, we presume all "commands" are loaded.
        self._ready_attrs = {'commands'}

        # Default parameters when calling wait_ready() or wait_ready(True).
        self._default_ready_attrs = ['armed', 'mode']

        @self.on_attribute('*')
        def listener(_, name, value):
            self._ready_attrs.add(name)

        # Attaches message listeners.
        self._message_listeners = dict()

        @handler.forward_message
        def listener(_, msg):
            self.notify_message_listeners(msg.get_type(), msg)

        self._location = Locations(self)

        @self.on_message('STATUSTEXT')
        def listener_STATUSTEXT(self, name, m):
            # Log the STATUSTEXT on the autopilot logger, with the correct severity
            self._autopilot_logger.log(
                msg=m.text.strip(),
                level=self._mavlink_statustext_severity[m.severity]
            )


        self._trueAirspeed = None
        self._trueTrimSpeed = None
        self._trueAirspeedMultiplier = None

        @self.on_message('SWOOP_AIRSPEED')
        def listener(self, name, m):
            self._trueAirspeed = m.trueAirspeed / 100 #convert from cm/s to m/s
            self._trueTrimSpeed = m.trueTrimSpeed / 100 #convert from cm/s to m/s
            self._trueAirspeedMultiplier = m.trueAirspeed


        self._heading = None
        self._airspeed = None
        self._groundspeed = None

        @self.on_message('VFR_HUD')
        def listener(self, name, m):
            self._heading = m.heading
            self._airspeed = m.airspeed            
            self._groundspeed = m.groundspeed

        self._rngfnd_distance = None
        self._rngfnd_voltage = None

        @self.on_message('RANGEFINDER')
        def listener(self, name, m):
            self._rngfnd_distance = m.distance
            self._rngfnd_voltage = m.voltage


        self.climb = None

        @self.on_message('VFR_HUD')
        def listener_VFR_HUD(self, name, m):
            self.climb = round(m.climb)

        self.windDetails = None
        self.savedWindDetails = None
        @self.on_message('WIND')
        def listener_WIND(self, name, m):
            self.windDetails = m

            if (self._swoopstatus is not None and SwoopStatus(self._swoopstatus).statusText() in ['FORWARD_FLIGHT','EMERGENCY_LAND']):
                self.savedWindDetails = m

        self._track = None
        @self.on_message('GPS_RAW_INT')
        def listener_GPS_RAW_INT(self, name, m):
            self._track = m.cog/100

        self.pixhawktemp = None

        @self.on_message('SENSOR_OFFSETS')
        def listener_SENSOR_OFFSETS(self, name, m):
            self.pixhawktemp = m.raw_temp / 100

        self.battery2_level = None

        @self.on_message('BATTERY_STATUS')
        def listener_BATTERY_STATUS(self, name, m):
            self.battery2_level = m.battery_remaining

        self.battery2_voltage = None
        self.battery2_current = None

        @self.on_message('BATTERY2')
        def listener_BATTERY2(self, name, m):
            self.battery2_voltage = m.voltage/1000
            self.battery2_current = m.current_battery


        self._swoop_arming_flags = None
        @self.on_message('SWOOP_ARMING_FLAGS')
        def listener_SWOOP_ARMING_FLAGS(self, name, m):
            self._swoop_arming_flags = m

        self._swoop_inflight_flags = None
        @self.on_message('SWOOP_INFLIGHT_FLAGS_INSTANT')
        def listener_SWOOP_INFLIGHT_FLAGS_INSTANT(self, name, m):            
            self._swoop_inflight_flags = m

        self._swoopstatus = None
        @self.on_message('SWOOP_STATUS')
        def listener_SWOOP_STATUS(self, name, m):
            self._swoopstatus = m



        self.ForwardEndurance = None
        self.ForwardWHrPortionRemaining = None
        self.ForwardHealth = None
        self.HoverEndurance = None
        self.HoverHealth = None
        self.HoverWHrPortionRemaining = None
        self.ForwardEnduranceReserve = None
        self.etr = {}

        @self.on_message('SWOOP_ENERGY')
        def listener_SWOOP_ENERGY(self, name, m):
            fixedFuelReserve = 18.8
            if m.ForwardWHrPortionRemaining == 0:
                self.ForwardEnduranceReserve = 0
            else:
                self.ForwardEnduranceReserve = round((m.ForwardEndurance / (m.ForwardWHrPortionRemaining)) * fixedFuelReserve)
            self.ForwardEndurance = m.ForwardEndurance - self.ForwardEnduranceReserve

            self.ForwardWHrPortionRemaining = m.ForwardWHrPortionRemaining
            self.ForwardHealth = m.ForwardHealth

            self.HoverEndurance = m.HoverEndurance
            self.HoverHealth = m.HoverHealth
            self.HoverWHrPortionRemaining = m.HoverWHrPortionRemaining

            self.etr["NextLanding"] = m.ForwardTimeToNextLanding
            self.etr["EndOfMission"] = m.ForwardTimeToEndOfMission
            self.etr["NextLanding"] = m.HoverTimeToNextLanding
            self.etr["EndOfMission"] = m.HoverTimeToEndOfMission



        self._capabilities = None
        self._raw_version = None
        self._autopilot_version_msg_count = 0
        self._autopilotVersion = {}
        @self.on_message('AUTOPILOT_VERSION')
        def listener(vehicle, name, m):
            self._capabilities = m.capabilities
            self._raw_version = m.flight_sw_version
            self.autopilotVersion = {}
            self._autopilotVersion['flight'] = ''.join(chr(i) for i in m.flight_custom_version).replace('\u0000','0')
            self._autopilotVersion['middleware'] = ''.join(chr(i) for i in m.middleware_custom_version)
            self._autopilotVersion['os'] = ''.join(chr(i) for i in m.os_custom_version)

            self._autopilot_version_msg_count += 1
            if self._capabilities != 0 or self._autopilot_version_msg_count > 5:
                # ArduPilot <3.4 fails to send capabilities correctly
                # straight after boot, and even older versions send
                # this back as always-0.
                vehicle.remove_message_listener('HEARTBEAT', self.send_capabilities_request)
            self.notify_attribute_listeners('autopilot_version', self._raw_version)


        @self.on_message('SYS_STATUS')
        def listener(self, name, m):
            self._voltage = m.voltage_battery
            self._current = m.current_battery
            self._level = m.battery_remaining
            self.notify_attribute_listeners('battery', self.battery)


        self._current_waypoint = 0
        @self.on_message(['WAYPOINT_CURRENT', 'MISSION_CURRENT'])
        def listener(self, name, m):
            self._current_waypoint = m.seq


        self._flightmode = 'AUTO'
        self._armed = False
        self._system_status = None
        self._autopilot_type = None  # PX4, ArduPilot, etc.
        self._vehicle_type = None  # quadcopter, plane, etc.

        @self.on_message('HEARTBEAT')
        def listener(self, name, m):
            # ignore groundstations
            if m.type == mavutil.mavlink.MAV_TYPE_GCS:
                return
            self._armed = (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            self.notify_attribute_listeners('armed', self.armed, cache=True)
            self._autopilot_type = m.autopilot
            self._vehicle_type = m.type
            if self._is_mode_available(m.custom_mode, m.base_mode) is False:
                raise APIException("mode (%s, %s) not available on mavlink definition" % (m.custom_mode, m.base_mode))
            if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                self._flightmode = mavutil.interpret_px4_mode(m.base_mode, m.custom_mode)
            else:
                self._flightmode = self._mode_mapping_bynumber[m.custom_mode]
            self.notify_attribute_listeners('mode', self.mode, cache=True)
            self._system_status = m.system_status

        # Waypoints.

        self._home_location = None
        self._wploader = mavwp.MAVWPLoader()
        self._wp_loaded = True
        self._wp_uploaded = None
        self._wpts_dirty = False
        self._commands = CommandSequence(self)

        @self.on_message(['WAYPOINT_COUNT', 'MISSION_COUNT'])
        def listener(self, name, msg):
            self._logger.debug("------------------------------RECEIVE --- " + str(msg))
            if not self._wp_loaded:
                self._wploader.clear()
                self._wploader.expected_count = msg.count
                self._master.waypoint_request_send(0)

        @self.on_message(['WAYPOINT', 'MISSION_ITEM'])
        def listener(self, name, msg):
            self._logger.debug("------------------------------RECEIVE MISSION ITEM --- " + str(msg))
            if not self._wp_loaded:
                if msg.seq == 0:
                    if not (msg.x == 0 and msg.y == 0 and msg.z == 0):
                        self._home_location = LocationGlobal(msg.x, msg.y, msg.z)

                if msg.seq > self._wploader.count():
                    # Unexpected waypoint
                    pass
                elif msg.seq < self._wploader.count():
                    # Waypoint duplicate
                    pass
                else:
                    self._wploader.add(msg)

                    if msg.seq + 1 < self._wploader.expected_count:
                        self._logger.debug("------------------------------SEND REQUEST   --- " + str(msg.seq))
                        self._master.waypoint_request_send(msg.seq + 1)
                    else:
                        self._wp_loaded = True
                        self.notify_attribute_listeners('commands', self.commands)

        # Waypoint send to master
        @self.on_message(['WAYPOINT_REQUEST', 'MISSION_REQUEST'])
        def listener(self, name, msg):
            self._logger.debug("RECEIVE MISSION REQUEST --- " + str(msg))
            if self._wp_uploaded is not None:
                wp = self._wploader.wp(msg.seq)
                handler.fix_targets(wp)
                self._logger.debug("SEND ITEM  --- " + str(wp))
                self._master.mav.send(wp)
                self._wp_uploaded[msg.seq] = True

        # TODO: Waypoint loop listeners

        # Parameters.

        start_duration = 0.2
        repeat_duration = 1

        self._params_count = -1
        self._params_set = []
        self._params_list = []
        self._params_downloaded_list = []

        self._params_loaded = True
        self._params_start = False
        self._params_map = {}
        self._params_last = monotonic.monotonic()  # Last new param.
        self._params_duration = start_duration
        self._parameters = Parameters(self)

        # @handler.forward_loop
        # def listener(_):
        #     # Check the time duration for last "new" params exceeds watchdog.
        #     if not self._params_start:
        #         return

        #     if not self._params_loaded and all(x is not None for x in self._params_set):
        #         self._params_loaded = True
        #         self.notify_attribute_listeners('parameters', self.parameters)

        #     if not self._params_loaded and monotonic.monotonic() - self._params_last > self._params_duration:
        #         c = 0
        #         for i, v in enumerate(self._params_set):
        #             if v is None:
        #                 self._master.mav.param_request_read_send(0, 0, b'', i)
        #                 c += 1
        #                 if c > 50:
        #                     break
        #         self._params_duration = repeat_duration
        #         self._params_last = monotonic.monotonic()

        @self.on_message(['PARAM_VALUE'])
        def listener(self, name, msg):
            # If we discover a new param count, assume we
            # are receiving a new param set.
            try:
                if str(msg.param_id) in self._params_list:
                    #self._logger.info("Parameter Received - Saved: " + msg.param_id + " -- " + str(msg))
                    #self._params_set[msg.param_index] = msg
                    self._params_map[msg.param_id] = msg.param_value
                    self._parameters.notify_attribute_listeners(msg.param_id, msg.param_value,cache=True)
                    self._params_downloaded_list.append(msg.param_id)
                else:
                    pass
                    #self._logger.info("Parameter Received - Ignored: " + msg.param_id + " -- " + str(msg))

            except:
                import traceback
                traceback.print_exc()

        # Heartbeats.

        self._heartbeat_started = False
        self._heartbeat_lastsent = 0
        self._heartbeat_lastreceived = 0
        self._heartbeat_timeout = False

        self._heartbeat_warning = 5
        self._heartbeat_error = 60
        self._heartbeat_system = None

        @handler.forward_loop
        def listener(_):
            # Send 1 heartbeat per second
            if monotonic.monotonic() - self._heartbeat_lastsent > 1:
                self._master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self._heartbeat_lastsent = monotonic.monotonic()

            # Timeouts.
            if self._heartbeat_started:
                if self._heartbeat_error and monotonic.monotonic() - self._heartbeat_lastreceived > self._heartbeat_error > 0:
                    raise APIException('No heartbeat in %s seconds, aborting.' %
                                       self._heartbeat_error)
                elif monotonic.monotonic() - self._heartbeat_lastreceived > self._heartbeat_warning:
                    if self._heartbeat_timeout is False:
                        self._logger.warning('Link timeout, no heartbeat in last %s seconds' % self._heartbeat_warning)
                        self._heartbeat_timeout = True

        self._lastHeartbeatTime = None

        @self.on_message(['HEARTBEAT'])
        def listener(self, name, msg):
            # ignore groundstations
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                return
            self._heartbeat_system = msg.get_srcSystem()
            self._heartbeat_lastreceived = monotonic.monotonic()
            self._lastHeartbeatTime = int(round(time.time() * 1000))
            if self._heartbeat_timeout:
                self._logger.info('...link restored.')
            self._heartbeat_timeout = False

        self._last_heartbeat = None

        @handler.forward_loop
        def listener(_):
            if self._heartbeat_lastreceived:
                self._last_heartbeat = monotonic.monotonic() - self._heartbeat_lastreceived
                self.notify_attribute_listeners('last_heartbeat', self.last_heartbeat)

    @property
    def last_heartbeat(self):
        """
        Time since last MAVLink heartbeat was received (in seconds).

        The attribute can be used to monitor link activity and implement script-specific timeout handling.

        For example, to pause the script if no heartbeat is received for more than 1 second you might implement
        the following observer, and use ``pause_script`` in a program loop to wait until the link is recovered:

        .. code-block:: python

            pause_script=False

            @vehicle.on_attribute('last_heartbeat')
            def listener(self, attr_name, value):
                global pause_script
                if value > 1 and not pause_script:
                    print "Pausing script due to bad link"
                    pause_script=True;
                if value < 1 and pause_script:
                    pause_script=False;
                    print "Un-pausing script"

        The observer will be called at the period of the messaging loop (about every 0.01 seconds). Testing
        on SITL indicates that ``last_heartbeat`` averages about .5 seconds, but will rarely exceed 1.5 seconds
        when connected. Whether heartbeat monitoring can be useful will very much depend on the application.


        .. note::

            If you just want to change the heartbeat timeout you can modify the ``heartbeat_timeout``
            parameter passed to the :py:func:`connect() <dronekit.connect>` function.

        """
        return self._last_heartbeat

    def getParameters(self, paramRequestList):
        self._params_set = [None] * len(paramRequestList)
        self._params_list = paramRequestList
        self._logger.info("Downloading Parameters: " + ', '.join(paramRequestList))

        for param in paramRequestList:     # First Example
            i = 0
            fetched = False
            while (i < 5 and not fetched):
                self._master.param_fetch_one(param)
                time.sleep(0.5)
                i += 1
                if param in self._params_downloaded_list:
                    fetched = True
        return

    def on_message(self, name):
        """
        Decorator for message listener callback functions.

        .. tip::

            This is the most elegant way to define message listener callback functions.
            Use :py:func:`add_message_listener` only if you need to be able to
            :py:func:`remove the listener <remove_message_listener>` later.

        A decorated message listener function is called with three arguments every time the
        specified message is received:

        * ``self`` - the current vehicle.
        * ``name`` - the name of the message that was intercepted.
        * ``message`` - the actual message (a `pymavlink <http://www.qgroundcontrol.org/mavlink/pymavlink>`_
          `class <https://www.samba.org/tridge/UAV/pymavlink/apidocs/classIndex.html>`_).

        For example, in the fragment below ``my_method`` will be called for every heartbeat message:

        .. code:: python

            @vehicle.on_message('HEARTBEAT')
            def my_method(self, name, msg):
                pass

        See :ref:`mavlink_messages` for more information.

        :param String name: The name of the message to be intercepted by the decorated listener function (or '*' to get all messages).
        """

        def decorator(fn):
            if isinstance(name, list):
                for n in name:
                    self.add_message_listener(n, fn)
            else:
                self.add_message_listener(name, fn)

        return decorator

    def add_message_listener(self, name, fn):
        """
        Adds a message listener function that will be called every time the specified message is received.

        .. tip::

            We recommend you use :py:func:`on_message` instead of this method as it has a more elegant syntax.
            This method is only preferred if you need to be able to
            :py:func:`remove the listener <remove_message_listener>`.

        The callback function must have three arguments:

        * ``self`` - the current vehicle.
        * ``name`` - the name of the message that was intercepted.
        * ``message`` - the actual message (a `pymavlink <http://www.qgroundcontrol.org/mavlink/pymavlink>`_
          `class <https://www.samba.org/tridge/UAV/pymavlink/apidocs/classIndex.html>`_).

        For example, in the fragment below ``my_method`` will be called for every heartbeat message:

        .. code:: python

            #Callback method for new messages
            def my_method(self, name, msg):
                pass

            vehicle.add_message_listener('HEARTBEAT',my_method)

        See :ref:`mavlink_messages` for more information.

        :param String name: The name of the message to be intercepted by the listener function (or '*' to get all messages).
        :param fn: The listener function that will be called if a message is received.
        """
        name = str(name)
        if name not in self._message_listeners:
            self._message_listeners[name] = []
        if fn not in self._message_listeners[name]:
            self._message_listeners[name].append(fn)

    def remove_message_listener(self, name, fn):
        """
        Removes a message listener (that was previously added using :py:func:`add_message_listener`).

        See :ref:`mavlink_messages` for more information.

        :param String name: The name of the message for which the listener is to be removed (or '*' to remove an 'all messages' observer).
        :param fn: The listener callback function to remove.

        """
        name = str(name)
        if name in self._message_listeners:
            if fn in self._message_listeners[name]:
                self._message_listeners[name].remove(fn)
                if len(self._message_listeners[name]) == 0:
                    del self._message_listeners[name]

    def notify_message_listeners(self, name, msg):
        for fn in self._message_listeners.get(name, []):
            try:
                fn(self, name, msg)
            except Exception:
                self._logger.exception('Exception in message handler for %s' % msg.get_type(), exc_info=True)

        for fn in self._message_listeners.get('*', []):
            try:
                fn(self, name, msg)
            except Exception:
                self._logger.exception('Exception in message handler for %s' % msg.get_type(), exc_info=True)

    def close(self):
        return self._handler.close()

    def flush(self):
        """
        Call ``flush()`` after :py:func:`adding <CommandSequence.add>` or :py:func:`clearing <CommandSequence.clear>` mission commands.

        After the return from ``flush()`` any writes are guaranteed to have completed (or thrown an
        exception) and future reads will see their effects.

        .. warning::

            This method is deprecated. It has been replaced by
            :py:func:`Vehicle.commands.upload() <Vehicle.commands.upload>`.
        """
        return self.commands.upload()

    #
    # Private sugar methods
    #

    @property
    def _mode_mapping(self):
        return self._master.mode_mapping()

    @property
    def _mode_mapping_bynumber(self):
        return mavutil.mode_mapping_bynumber(self._vehicle_type)

    def _is_mode_available(self, custommode_code, basemode_code=0):
        try:
            if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                mode = mavutil.interpret_px4_mode(basemode_code, custommode_code)
                return mode in self._mode_mapping
            return custommode_code in self._mode_mapping_bynumber
        except:
            return False

    #
    # Operations to support the standard API.
    #

    @property
    def mode(self):
        """
        This attribute is used to get and set the current flight mode. You
        can specify the value as a :py:class:`VehicleMode`, like this:

        .. code-block:: python

           vehicle.mode = VehicleMode('LOITER')

        Or as a simple string:

        .. code-block:: python

            vehicle.mode = 'LOITER'

        If you are targeting ArduPilot you can also specify the flight mode
        using a numeric value (this will not work with PX4 autopilots):

        .. code-block:: python

            # set mode to LOITER
            vehicle.mode = 5
        """
        if not self._flightmode:
            return None
        return VehicleMode(self._flightmode)

    @mode.setter
    def mode(self, v):
        if isinstance(v, basestring):
            v = VehicleMode(v)

        if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            self._master.set_mode(v.name)
        elif isinstance(v, int):
            self._master.set_mode(v)
        else:
            self._master.set_mode(self._mode_mapping[v.name])

    @property
    def swoopstatus(self):
        """
        SwoopStatus (:py:class:`SwoopStatus`).
        """
        return SwoopStatus(self._swoopstatus)

    @property
    def position(self):
        """
        SwoopPosition (:py:class:`SwoopPosition`).
        """
        return Position(self._location.global_frame.lat, self._location.global_frame.lon, self._location.global_frame.alt, self._rngfnd_distance, self._track, self._heading)

    @property
    def speed(self):
        """
        Speed (:py:class:`Speed`).
        """
        return Speed(self._groundspeed, self._airspeed, self._trueAirspeed, self._trueTrimSpeed,  self.climb)
    
    
    # @property 
    # def speedtas(self):
    #     speedtas = {}
    #     speedtas['TAS'] = self._trueAirspeed
    #     speedtas['forwardSetTAS'] = self._trueTrimSpeed
    #     speedtas['airspeedMultiplierTAS'] = self._trueAirspeedMultiplier
    #     return speedtas

    @property
    def battery(self):
        """
        Current system batter status (:py:class:`Battery`).
        """
        if self._voltage is None or self._current is None or self._level is None:
            return None
        return Battery(self._voltage, self._current, self._level)

    @property
    def winddetails(self):
        return WindDetails(SwoopStatus(self._swoopstatus).statusText(), self.windDetails, self.savedWindDetails)

    @property
    def batteryhv(self):
        if self.battery2_voltage is not None:
            battery = {}
            battery['voltage'] = round(self.battery2_voltage,2)
            battery['current'] = round(self.battery2_current)
            battery['level'] = self.battery2_level

            battery['endurance'] = self.HoverEndurance
            battery['remaining'] = self.HoverWHrPortionRemaining
            battery['health'] = self.HoverHealth
            return battery
        else:
            return {}


    @property
    def autopilotversion(self):
        autopilotVersion = self._autopilotVersion
        autopilotVersion["name"]= str(self.version)
        return autopilotVersion

    @property
    def batteryfw(self):
        battery = {}
        battery['voltage'] = round(self.battery.voltage,2)
        if self.battery.current is not None:
            battery['current'] = round(self.battery.current)
        if self.battery.level is not None:
            battery['level'] = self.battery.level

        battery['endurance'] = self.ForwardEndurance
        battery['endurancereserve'] = self.ForwardEnduranceReserve
        battery['remaining'] = self.ForwardWHrPortionRemaining
        battery['health'] = self.ForwardHealth
        return battery
        
    @property
    def sensorsoffsets(self):
        if self.pixhawktemp is not None:
            return SensorOffsets(self.pixhawktemp)
    
    @property
    def flags(self):
        if self._swoop_inflight_flags is not None:
            return SwoopInFlightFlags(self._swoop_inflight_flags)

    @property
    def ready(self):
        if self._swoop_arming_flags is not None:
            return SwoopArmingFlags(self._swoop_arming_flags)

    @property
    def swoopstatus(self):
        if self._swoopstatus is not None:
            return SwoopStatus(self._swoopstatus)

    @property
    def version(self):
        """
        The autopilot version and type in a :py:class:`Version` object.

        .. versionadded:: 2.0.3
        """
        return Version(self._raw_version, self._autopilot_type, self._vehicle_type)

    @property
    def capabilities(self):
        """
        The autopilot capabilities in a :py:class:`Capabilities` object.

        .. versionadded:: 2.0.3
        """
        return Capabilities(self._capabilities)


    @property
    def armed(self):
        """
        This attribute can be used to get and set the ``armed`` state of the vehicle (``boolean``).

        The code below shows how to read the state, and to arm/disarm the vehicle:

        .. code:: python

            # Print the armed state for the vehicle
            print "Armed: %s" % vehicle.armed

            # Disarm the vehicle
            vehicle.armed = False

            # Arm the vehicle
            vehicle.armed = True
        """
        return self._armed

    @armed.setter
    def armed(self, value):
        if bool(value) != self._armed:
            if value:
                self._master.arducopter_arm()
            else:
                self._master.arducopter_disarm()


    @property
    def lastHeartbeatTime(self):
        return self._lastHeartbeatTime



    @property
    def commands(self):
        """
        Gets the editable waypoints/current mission for this vehicle (:py:class:`CommandSequence`).

        This can be used to get, create, and modify a mission.

        :returns: A :py:class:`CommandSequence` containing the waypoints for this vehicle.
        """
        return self._commands

    @property
    def parameters(self):
        """
        The (editable) parameters for this vehicle (:py:class:`Parameters`).
        """
        return self._parameters

    def wait_for(self, condition, timeout=None, interval=0.1, errmsg=None):
        '''Wait for a condition to be True.

        Wait for condition, a callable, to return True.  If timeout is
        nonzero, raise a TimeoutError(errmsg) if the condition is not
        True after timeout seconds.  Check the condition everal
        interval seconds.
        '''

        t0 = time.time()
        while not condition():
            t1 = time.time()
            if timeout and (t1 - t0) >= timeout:
                raise TimeoutError(errmsg)

            time.sleep(interval)


    def arm(self, wait=True, timeout=None):
        '''Arm the vehicle.

        If wait is True, wait for arm operation to complete before
        returning.  If timeout is nonzero, raise a TimeouTerror if the
        vehicle has not armed after timeout seconds.
        '''

        self.armed = True

        if wait:
            self.wait_for(lambda: self.armed, timeout=timeout,
                          errmsg='failed to arm vehicle')

    def disarm(self, wait=True, timeout=None):
        '''Disarm the vehicle.

        If wait is True, wait for disarm operation to complete before
        returning.  If timeout is nonzero, raise a TimeouTerror if the
        vehicle has not disarmed after timeout seconds.
        '''
        self.armed = False

        if wait:
            self.wait_for(lambda: not self.armed, timeout=timeout,
                          errmsg='failed to disarm vehicle')


    def wait_for_mode(self, mode, timeout=None):
        '''Set the flight mode.

        If wait is True, wait for the mode to change before returning.
        If timeout is nonzero, raise a TimeoutError if the flight mode
        hasn't changed after timeout seconds.
        '''

        if not isinstance(mode, VehicleMode):
            mode = VehicleMode(mode)

        self.mode = mode

        self.wait_for(lambda: self.mode.name == mode.name,
                      timeout=timeout,
                      errmsg='failed to set flight mode')


    def send_mavlink(self, message):
        """
        This method is used to send raw MAVLink "custom messages" to the vehicle.

        The function can send arbitrary messages/commands to the connected vehicle at any time and in any vehicle mode.
        It is particularly useful for controlling vehicles outside of missions (for example, in GUIDED mode).

        The :py:func:`message_factory <dronekit.Vehicle.message_factory>` is used to create messages in the appropriate format.

        For more information see the guide topic: :ref:`guided_mode_how_to_send_commands`.

        :param message: A ``MAVLink_message`` instance, created using :py:func:`message_factory <dronekit.Vehicle.message_factory>`.
            There is need to specify the system id, component id or sequence number of messages as the API will set these appropriately.
        """
        self._master.mav.send(message)

    @property
    def message_factory(self):
        """
        Returns an object that can be used to create 'raw' MAVLink messages that are appropriate for this vehicle.
        The message can then be sent using :py:func:`send_mavlink(message) <dronekit.Vehicle.send_mavlink>`.

        .. note::

            Vehicles support a subset of the messages defined in the MAVLink standard. For more information
            about the supported sets see wiki topics:
            `Copter Commands in Guided Mode <http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/>`_
            and `Plane Commands in Guided Mode <http://dev.ardupilot.com/wiki/plane-commands-in-guided-mode/>`_.

        All message types are defined in the central MAVLink github repository.  For example, a Pixhawk understands
        the following messages (from `pixhawk.xml <https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/pixhawk.xml>`_):

        .. code:: xml

          <message id="153" name="IMAGE_TRIGGER_CONTROL">
               <field type="uint8_t" name="enable">0 to disable, 1 to enable</field>
          </message>

        The name of the factory method will always be the lower case version of the message name with *_encode* appended.
        Each field in the XML message definition must be listed as arguments to this factory method.  So for this example
        message, the call would be:

        .. code:: python

            msg = vehicle.message_factory.image_trigger_control_encode(True)
            vehicle.send_mavlink(msg)

        Some message types include "addressing information". If present, there is no need to specify the ``target_system``
        id (just set to zero) as DroneKit will automatically update messages with the correct ID for the connected
        vehicle before sending.
        The ``target_component`` should be set to 0 (broadcast) unless the message is to specific component.
        CRC fields and sequence numbers (if defined in the message type) are automatically set by DroneKit and can also
        be ignored/set to zero.

        For more information see the guide topic: :ref:`guided_mode_how_to_send_commands`.
        """
        return self._master.mav

    def initialize(self, rate=4, heartbeat_timeout=30):
        self._handler.start()

        # Start heartbeat polling.
        start = monotonic.monotonic()
        self._heartbeat_error = heartbeat_timeout or 0
        self._heartbeat_started = True
        self._heartbeat_lastreceived = start

        # Poll for first heartbeat.
        # If heartbeat times out, this will interrupt.
        while self._handler._alive:
            time.sleep(.1)
            if self._heartbeat_lastreceived != start:
                break
        if not self._handler._alive:
            raise APIException('Timeout in initializing connection.')

        # Register target_system now.
        self._handler.target_system = self._heartbeat_system

        # Wait until board has booted.
        while True:
            if self._flightmode not in [None, 'INITIALISING', 'MAV']:
                break
            time.sleep(0.1)

        # Initialize data stream.
        if rate is not None:
            self._master.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                                      rate, 1)

        self.add_message_listener('HEARTBEAT', self.send_capabilities_request)

        # # Ensure initial parameter download has started.
        # while True:
        #     # This fn actually rate limits itself to every 2s.
        #     # Just retry with persistence to get our first param stream.
        #     self._master.param_fetch_all()
        #     time.sleep(0.1)
        #     if self._params_count > -1:
        #         break

    def send_capabilties_request(self, vehicle, name, m):
        '''An alias for send_capabilities_request.

        The word "capabilities" was misspelled in previous versions of this code. This is simply
        an alias to send_capabilities_request using the legacy name.
        '''
        return self.send_capabilities_request(vehicle, name, m)

    def send_capabilities_request(self, vehicle, name, m):
        '''Request an AUTOPILOT_VERSION packet'''
        capability_msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 0, 1, 0, 0, 0, 0, 0, 0)
        vehicle.send_mavlink(capability_msg)

    def play_tune(self, tune):
        '''Play a tune on the vehicle'''
        msg = self.message_factory.play_tune_encode(0, 0, tune)
        self.send_mavlink(msg)

    def wait_ready(self, *types, **kwargs):
        """
        Waits for specified attributes to be populated from the vehicle (values are initially ``None``).

        This is typically called "behind the scenes" to ensure that :py:func:`connect` does not return until
        attributes have populated (via the ``wait_ready`` parameter). You can also use it after connecting to
        wait on a specific value(s).

        There are two ways to call the method:

        .. code-block:: python

            #Wait on default attributes to populate
            vehicle.wait_ready(True)

            #Wait on specified attributes (or array of attributes) to populate
            vehicle.wait_ready('mode','airspeed')

        Using the ``wait_ready(True)`` waits on :py:attr:`parameters`, :py:attr:`gps_0`,
        :py:attr:`armed`, :py:attr:`mode`, and :py:attr:`attitude`. In practice this usually
        means that all supported attributes will be populated.

        By default, the method will timeout after 30 seconds and raise an exception if the
        attributes were not populated.

        :param types: ``True`` to wait on the default set of attributes, or a
            comma-separated list of the specific attributes to wait on.
        :param int timeout: Timeout in seconds after which the method will raise an exception
            (the default) or return ``False``. The default timeout is 30 seconds.
        :param Boolean raise_exception: If ``True`` the method will raise an exception on timeout,
            otherwise the method will return ``False``. The default is ``True`` (raise exception).
        """

        #for key, value in kwargs.items():
        #    self._logger.info('Vehicle wait_ready - ' + str(key) + ": " + str(value))

        timeout = kwargs.get('timeout', 30)

        #self._logger.info("Connect Wait timeout: " + str(timeout))
        raise_exception = kwargs.get('raise_exception', True)

        # Vehicle defaults for wait_ready(True) or wait_ready()
        if list(types) == [True] or list(types) == []:
            types = self._default_ready_attrs

        if not all(isinstance(item, basestring) for item in types):
            raise ValueError('wait_ready expects one or more string arguments.')

        # Wait for these attributes to have been set.
        await_attributes = set(types)
        start = monotonic.monotonic()
        still_waiting_last_message_sent = start
        still_waiting_callback = kwargs.get('still_waiting_callback')
        still_waiting_message_interval = kwargs.get('still_waiting_interval', 1)

        while not await_attributes.issubset(self._ready_attrs):
            time.sleep(0.1)
            now = monotonic.monotonic()
            if now - start > timeout:
                if raise_exception:
                    raise TimeoutError('wait_ready experienced a timeout after %s seconds.' %
                                       timeout)
                else:
                    return False
            if (still_waiting_callback and
                    now - still_waiting_last_message_sent > still_waiting_message_interval):
                still_waiting_last_message_sent = now
                if still_waiting_callback:
                    still_waiting_callback(await_attributes - self._ready_attrs)

        return True

    def reboot(self):
        """Requests an autopilot reboot by sending a ``MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`` command."""

        reboot_msg = self.message_factory.command_long_encode(
            0, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,  # command
            0,  # confirmation
            1,  # param 1, autopilot (reboot)
            0,  # param 2, onboard computer (do nothing)
            0,  # param 3, camera (do nothing)
            0,  # param 4, mount (do nothing)
            0, 0, 0)  # param 5 ~ 7 not used

        self.send_mavlink(reboot_msg)

    def send_calibrate_gyro(self):
        """Request gyroscope calibration."""

        calibration_command = self.message_factory.command_long_encode(
            self._handler.target_system, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
            0,  # confirmation
            1,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
            0,  # param 2, 1: magnetometer calibration
            0,  # param 3, 1: ground pressure calibration
            0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
            0,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
            0,  # param 6, 2: airspeed calibration
            0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
        )
        self.send_mavlink(calibration_command)

    def send_calibrate_magnetometer(self):
        """Request magnetometer calibration."""

        # ArduPilot requires the MAV_CMD_DO_START_MAG_CAL command, only present in the ardupilotmega.xml definition
        if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            calibration_command = self.message_factory.command_long_encode(
                self._handler.target_system, 0,  # target_system, target_component
                mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,  # command
                0,  # confirmation
                0,  # param 1, uint8_t bitmask of magnetometers (0 means all).
                1,  # param 2, Automatically retry on failure (0=no retry, 1=retry).
                1,  # param 3, Save without user input (0=require input, 1=autosave).
                0,  # param 4, Delay (seconds).
                0,  # param 5, Autoreboot (0=user reboot, 1=autoreboot).
                0,  # param 6, Empty.
                0,  # param 7, Empty.
            )
        else:
            calibration_command = self.message_factory.command_long_encode(
                self._handler.target_system, 0,  # target_system, target_component
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
                0,  # confirmation
                0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
                1,  # param 2, 1: magnetometer calibration
                0,  # param 3, 1: ground pressure calibration
                0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
                0,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
                0,  # param 6, 2: airspeed calibration
                0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
            )

        self.send_mavlink(calibration_command)

    def send_calibrate_accelerometer(self, simple=False):
        """Request accelerometer calibration.

        :param simple: if True, perform simple accelerometer calibration
        """

        calibration_command = self.message_factory.command_long_encode(
            self._handler.target_system, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
            0,  # confirmation
            0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
            0,  # param 2, 1: magnetometer calibration
            0,  # param 3, 1: ground pressure calibration
            0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
            4 if simple else 1,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
            0,  # param 6, 2: airspeed calibration
            0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
        )
        self.send_mavlink(calibration_command)

    def send_calibrate_vehicle_level(self):
        """Request vehicle level (accelerometer trim) calibration."""

        calibration_command = self.message_factory.command_long_encode(
            self._handler.target_system, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
            0,  # confirmation
            0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
            0,  # param 2, 1: magnetometer calibration
            0,  # param 3, 1: ground pressure calibration
            0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
            2,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
            0,  # param 6, 2: airspeed calibration
            0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
        )
        self.send_mavlink(calibration_command)

    def send_calibrate_barometer(self):
        """Request barometer calibration."""

        calibration_command = self.message_factory.command_long_encode(
            self._handler.target_system, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
            0,  # confirmation
            0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
            0,  # param 2, 1: magnetometer calibration
            1,  # param 3, 1: ground pressure calibration
            0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
            0,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
            0,  # param 6, 2: airspeed calibration
            0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
        )
        self.send_mavlink(calibration_command)


class Parameters(collections.MutableMapping, HasObservers):
    """
    This object is used to get and set the values of named parameters for a vehicle. See the following links for information about
    the supported parameters for each platform: `Copter Parameters <http://copter.ardupilot.com/wiki/configuration/arducopter-parameters/>`_,
    `Plane Parameters <http://plane.ardupilot.com/wiki/arduplane-parameters/>`_, `Rover Parameters <http://rover.ardupilot.com/wiki/apmrover2-parameters/>`_.

    The code fragment below shows how to get and set the value of a parameter.

    .. code:: python

        # Print the value of the THR_MIN parameter.
        print "Param: %s" % vehicle.parameters['THR_MIN']

        # Change the parameter value to something different.
        vehicle.parameters['THR_MIN']=100

    It is also possible to observe parameters and to iterate the :py:attr:`Vehicle.parameters`.

    For more information see :ref:`the guide <vehicle_state_parameters>`.
    """

    def __init__(self, vehicle):
        super(Parameters, self).__init__()
        self._logger = logging.getLogger(__name__)
        self._vehicle = vehicle

    def __getitem__(self, name):
        name = name.upper()
        #self.wait_ready()
        return self._vehicle._params_map[name]

    def __setitem__(self, name, value):
        name = name.upper()
        #self.wait_ready()
        self.set(name, value)

    def __delitem__(self, name):
        raise APIException('Cannot delete value from parameters list.')

    def __len__(self):
        return len(self._vehicle._params_map)

    def __iter__(self):
        return self._vehicle._params_map.__iter__()

    def get(self, name, wait_ready=True):
        name = name.upper()
        if wait_ready:
            self.wait_ready()
        return self._vehicle._params_map.get(name, None)

    def set(self, name, value, retries=3, wait_ready=False):
        #if wait_ready:
        #    self.wait_ready()

        # TODO dumbly reimplement this using timeout loops
        # because we should actually be awaiting an ACK of PARAM_VALUE
        # changed, but we don't have a proper ack structure, we'll
        # instead just wait until the value itself was changed

        name = name.upper()
        # convert to single precision floating point number (the type used by low level mavlink messages)
        value = float(struct.unpack('f', struct.pack('f', value))[0])
        remaining = retries
        while True:
            self._vehicle._master.param_set_send(name, value)
            tstart = monotonic.monotonic()
            if remaining == 0:
                break
            remaining -= 1
            while monotonic.monotonic() - tstart < 1:
                if name in self._vehicle._params_map and self._vehicle._params_map[name] == value:
                    return True
                time.sleep(0.1)

        if retries > 0:
            self._logger.error("timeout setting parameter %s to %f" % (name, value))
        return False

    def wait_ready(self, **kwargs):
        """
        Block the calling thread until parameters have been downloaded
        """

        #for key, value in kwargs.items():
        #    self._logger.info('Parameters wait_ready - **kwargs' + str(key) + ": " + str(value))


        self._vehicle.wait_ready('parameters', **kwargs)

    def add_attribute_listener(self, attr_name, *args, **kwargs):
        """
        Add a listener callback on a particular parameter.

        The callback can be removed using :py:func:`remove_attribute_listener`.

        .. note::

            The :py:func:`on_attribute` decorator performs the same operation as this method, but with
            a more elegant syntax. Use ``add_attribute_listener`` only if you will need to remove
            the observer.

        The callback function is invoked only when the parameter changes.

        The callback arguments are:

        * ``self`` - the associated :py:class:`Parameters`.
        * ``attr_name`` - the parameter name. This can be used to infer which parameter has triggered
          if the same callback is used for watching multiple parameters.
        * ``msg`` - the new parameter value (so you don't need to re-query the vehicle object).

        The example below shows how to get callbacks for the ``THR_MIN`` parameter:

        .. code:: python

            #Callback function for the THR_MIN parameter
            def thr_min_callback(self, attr_name, value):
                print " PARAMETER CALLBACK: %s changed to: %s" % (attr_name, value)

            #Add observer for the vehicle's THR_MIN parameter
            vehicle.parameters.add_attribute_listener('THR_MIN', thr_min_callback)

        See :ref:`vehicle_state_observing_parameters` for more information.

        :param String attr_name: The name of the parameter to watch (or '*' to watch all parameters).
        :param args: The callback to invoke when a change in the parameter is detected.

        """
        attr_name = attr_name.upper()
        return super(Parameters, self).add_attribute_listener(attr_name, *args, **kwargs)

    def remove_attribute_listener(self, attr_name, *args, **kwargs):
        """
        Remove a paremeter listener that was previously added using :py:func:`add_attribute_listener`.

        For example to remove the ``thr_min_callback()`` callback function:

        .. code:: python

            vehicle.parameters.remove_attribute_listener('thr_min', thr_min_callback)

        See :ref:`vehicle_state_observing_parameters` for more information.

        :param String attr_name: The parameter name that is to have an observer removed (or '*' to remove an 'all attribute' observer).
        :param args: The callback function to remove.

        """
        attr_name = attr_name.upper()
        return super(Parameters, self).remove_attribute_listener(attr_name, *args, **kwargs)

    def notify_attribute_listeners(self, attr_name, *args, **kwargs):
        attr_name = attr_name.upper()
        return super(Parameters, self).notify_attribute_listeners(attr_name, *args, **kwargs)

    def on_attribute(self, attr_name, *args, **kwargs):
        """
        Decorator for parameter listeners.

        .. note::

            There is no way to remove a listener added with this decorator. Use
            :py:func:`add_attribute_listener` if you need to be able to remove
            the :py:func:`listener <remove_attribute_listener>`.

        The callback function is invoked only when the parameter changes.

        The callback arguments are:

        * ``self`` - the associated :py:class:`Parameters`.
        * ``attr_name`` - the parameter name. This can be used to infer which parameter has triggered
          if the same callback is used for watching multiple parameters.
        * ``msg`` - the new parameter value (so you don't need to re-query the vehicle object).

        The code fragment below shows how to get callbacks for the ``THR_MIN`` parameter:

        .. code:: python

            @vehicle.parameters.on_attribute('THR_MIN')
            def decorated_thr_min_callback(self, attr_name, value):
                print " PARAMETER CALLBACK: %s changed to: %s" % (attr_name, value)

        See :ref:`vehicle_state_observing_parameters` for more information.

        :param String attr_name: The name of the parameter to watch (or '*' to watch all parameters).
        :param args: The callback to invoke when a change in the parameter is detected.

        """
        attr_name = attr_name.upper()
        return super(Parameters, self).on_attribute(attr_name, *args, **kwargs)


class Command(mavutil.mavlink.MAVLink_mission_item_message):
    """
    A waypoint object.

    This object encodes a single mission item command. The set of commands that are supported
    by ArduPilot in Copter, Plane and Rover (along with their parameters) are listed in the wiki article
    `MAVLink Mission Command Messages (MAV_CMD) <http://planner.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/>`_.

    For example, to create a `NAV_WAYPOINT <http://planner.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_nav_waypoint>`_ command:

    .. code:: python

        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,-34.364114, 149.166022, 30)

    :param target_system: This can be set to any value
        (DroneKit changes the value to the MAVLink ID of the connected vehicle before the command is sent).
    :param target_component: The component id if the message is intended for a particular component within the target system
        (for example, the camera). Set to zero (broadcast) in most cases.
    :param seq: The sequence number within the mission (the autopilot will reject messages sent out of sequence).
        This should be set to zero as the API will automatically set the correct value when uploading a mission.
    :param frame: The frame of reference used for the location parameters (x, y, z). In most cases this will be
        ``mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT``, which uses the WGS84 global coordinate system for latitude and longitude, but sets altitude
        as relative to the home position in metres (home altitude = 0). For more information `see the wiki here
        <http://planner.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#frames_of_reference>`_.
    :param command: The specific mission command (e.g. ``mavutil.mavlink.MAV_CMD_NAV_WAYPOINT``). The supported commands (and command parameters
        are listed `on the wiki <http://planner.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/>`_.
    :param current: Set to zero (not supported).
    :param autocontinue: Set to zero (not supported).
    :param param1: Command specific parameter (depends on specific `Mission Command (MAV_CMD) <http://planner.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/>`_).
    :param param2: Command specific parameter.
    :param param3: Command specific parameter.
    :param param4: Command specific parameter.
    :param x: (param5) Command specific parameter used for latitude (if relevant to command).
    :param y: (param6) Command specific parameter used for longitude (if relevant to command).
    :param z: (param7) Command specific parameter used for altitude (if relevant). The reference frame for altitude depends on the ``frame``.

    """
    pass


class CommandSequence(object):
    """
    A sequence of vehicle waypoints (a "mission").

    Operations include 'array style' indexed access to the various contained waypoints.

    The current commands/mission for a vehicle are accessed using the :py:attr:`Vehicle.commands` attribute.
    Waypoints are not downloaded from vehicle until :py:func:`download()` is called.  The download is asynchronous;
    use :py:func:`wait_ready()` to block your thread until the download is complete.
    The code to download the commands from a vehicle is shown below:

    .. code-block:: python
        :emphasize-lines: 5-10

        #Connect to a vehicle object (for example, on com14)
        vehicle = connect('com14', wait_ready=True)

        # Download the vehicle waypoints (commands). Wait until download is complete.
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()

    The set of commands can be changed and uploaded to the client. The changes are not guaranteed to be complete until
    :py:func:`upload() <Vehicle.commands.upload>` is called.

    .. code:: python

        cmds = vehicle.commands
        cmds.clear()
        lat = -34.364114,
        lon = 149.166022
        altitude = 30.0
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0, 0, 0,
            lat, lon, altitude)
        cmds.add(cmd)
        cmds.upload()

    """

    def __init__(self, vehicle):
        self._vehicle = vehicle

    def download(self):
        '''
        Download all waypoints from the vehicle.
        The download is asynchronous. Use :py:func:`wait_ready()` to block your thread until the download is complete.
        '''
        self.wait_ready()
        self._vehicle._ready_attrs.remove('commands')
        self._vehicle._wp_loaded = False
        self._vehicle._master.waypoint_request_list_send()
        # BIG FIXME - wait for full wpt download before allowing any of the accessors to work



    def restart_download(self):
        '''
        This restarts the download from the last known index if it missed a command
        '''
        #Pause and check count not changing:
        #logging.info("DOWNLOAD RESTARTED as " + str(self._vehicle._wploader.count()))

        initialCount = self._vehicle._wploader.count()
        time.sleep(1.5) #Possible error point. 1.5 based on the mavlink default timeout.
        if (initialCount == self._vehicle._wploader.count()):
            logging.info("Re Requesting Waypoint " + str(initialCount))
            self._vehicle._master.waypoint_request_send(initialCount)
            return True
        else:
            logging.info("Download Still going " + str(self._vehicle._wploader.count()) + " / " + str(self._vehicle._wploader.expected_count))
            return False

    def wait_ready(self, **kwargs):
        """
        Block the calling thread until waypoints have been downloaded.

        This can be called after :py:func:`download()` to block the thread until the asynchronous download is complete.
        """

        #for key, value in kwargs.items():
        #    logging.debug('Command Sequence wait_ready - **kwargs' + str(key) + ": " + str(value))

        return self._vehicle.wait_ready('commands', **kwargs)

    def clear(self):
        '''
        Clear the command list.

        This command will be sent to the vehicle only after you call :py:func:`upload() <Vehicle.commands.upload>`.
        '''

        # Add home point again.
        self.wait_ready()
        home = None
        try:
            home = self._vehicle._wploader.wp(0)
        except:
            pass
        self._vehicle._wploader.clear()
        if home:
            self._vehicle._wploader.add(home, comment='Added by DroneKit')
        self._vehicle._wpts_dirty = True

    def add(self, cmd):
        '''
        Add a new command (waypoint) at the end of the command list.

        .. note::

            Commands are sent to the vehicle only after you call ::py:func:`upload() <Vehicle.commands.upload>`.

        :param Command cmd: The command to be added.
        '''
        self.wait_ready()
        self._vehicle._handler.fix_targets(cmd)
        self._vehicle._wploader.add(cmd, comment='Added by DroneKit')
        self._vehicle._wpts_dirty = True

    def upload(self, timeout=None):
        """
        Call ``upload()`` after :py:func:`adding <CommandSequence.add>` or :py:func:`clearing <CommandSequence.clear>` mission commands.

        After the return from ``upload()`` any writes are guaranteed to have completed (or thrown an
        exception) and future reads will see their effects.

        :param int timeout: The timeout for uploading the mission. No timeout if not provided or set to None.
        """
        if self._vehicle._wpts_dirty:
            self._vehicle._master.waypoint_clear_all_send()
            start_time = time.time()
            if self._vehicle._wploader.count() > 0:
                self._vehicle._wp_uploaded = [False] * self._vehicle._wploader.count()
                self._vehicle._master.waypoint_count_send(self._vehicle._wploader.count())
                while False in self._vehicle._wp_uploaded:
                    if timeout and time.time() - start_time > timeout:
                        raise TimeoutError
                    time.sleep(0.1)
                self._vehicle._wp_uploaded = None
            self._vehicle._wpts_dirty = False

    @property
    def count(self):
        '''
        Return number of waypoints.

        :return: The number of waypoints in the sequence.
        '''
        return max(self._vehicle._wploader.count() - 1, 0)

    @property
    def next(self):
        """
        Get the currently active waypoint number.
        """
        return self._vehicle._current_waypoint

    @next.setter
    def next(self, index):
        """
        Set a new ``next`` waypoint for the vehicle.
        """
        self._vehicle._master.waypoint_set_current_send(index)

    def __len__(self):
        '''
        Return number of waypoints.

        :return: The number of waypoints in the sequence.
        '''
        return max(self._vehicle._wploader.count() - 1, 0)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return [self[ii] for ii in range(*index.indices(len(self)))]
        elif isinstance(index, int):
            item = self._vehicle._wploader.wp(index + 1)
            if not item:
                raise IndexError('Index %s out of range.' % index)
            return item
        else:
            raise TypeError('Invalid argument type.')

    def __setitem__(self, index, value):
        self._vehicle._wploader.set(value, index + 1)
        self._vehicle._wpts_dirty = True


def default_still_waiting_callback(atts):
    logging.getLogger(__name__).debug("Still waiting for data from vehicle: %s" % ','.join(atts))


def connect(ip,
            _initialize=True,
            wait_ready=None,
            timeout=30,
            still_waiting_callback=default_still_waiting_callback,
            still_waiting_interval=1,
            status_printer=None,
            vehicle_class=None,
            rate=4,
            baud=115200,
            heartbeat_timeout=30,
            source_system=255,
            source_component=0,
            use_native=False):
    """
    Returns a :py:class:`Vehicle` object connected to the address specified by string parameter ``ip``.
    Connection string parameters (``ip``) for different targets are listed in the :ref:`getting started guide <get_started_connecting>`.

    The method is usually called with ``wait_ready=True`` to ensure that vehicle parameters and (most) attributes are
    available when ``connect()`` returns.

    .. code:: python

        from dronekit import connect

        # Connect to the Vehicle using "connection string" (in this case an address on network)
        vehicle = connect('127.0.0.1:14550', wait_ready=True)

    :param String ip: :ref:`Connection string <get_started_connecting>` for target address - e.g. 127.0.0.1:14550.

    :param Bool/Array wait_ready: If ``True`` wait until all default attributes have downloaded before
        the method returns (default is ``None``).
        The default attributes to wait on are: :py:attr:`parameters`, :py:attr:`gps_0`,
        :py:attr:`armed`, :py:attr:`mode`, and :py:attr:`attitude`.

        You can also specify a named set of parameters to wait on (e.g. ``wait_ready=['system_status','mode']``).

        For more information see :py:func:`Vehicle.wait_ready <Vehicle.wait_ready>`.

    :param status_printer: (deprecated) method of signature ``def status_printer(txt)`` that prints
        STATUS_TEXT messages from the Vehicle and other diagnostic information.
        By default the status information is handled by the ``autopilot`` logger.
    :param Vehicle vehicle_class: The class that will be instantiated by the ``connect()`` method.
        This can be any sub-class of ``Vehicle`` (and defaults to ``Vehicle``).
    :param int rate: Data stream refresh rate. The default is 4Hz (4 updates per second).
    :param int baud: The baud rate for the connection. The default is 115200.
    :param int heartbeat_timeout: Connection timeout value in seconds (default is 30s).
        If a heartbeat is not detected within this time an exception will be raised.
    :param int source_system: The MAVLink ID of the :py:class:`Vehicle` object returned by this method (by default 255).
    :param int source_component: The MAVLink Component ID fo the :py:class:`Vehicle` object returned by this method (by default 0).
    :param bool use_native: Use precompiled MAVLink parser.

        .. note::

            The returned :py:class:`Vehicle` object acts as a ground control station from the
            perspective of the connected "real" vehicle. It will process/receive messages from the real vehicle
            if they are addressed to this ``source_system`` id. Messages sent to the real vehicle are
            automatically updated to use the vehicle's ``target_system`` id.

            It is *good practice* to assign a unique id for every system on the MAVLink network.
            It is possible to configure the autopilot to only respond to guided-mode commands from a specified GCS ID.

            The ``status_printer`` argument is deprecated. To redirect the logging from the library and from the
            autopilot, configure the ``dronekit`` and ``autopilot`` loggers using the Python ``logging`` module.


    :returns: A connected vehicle of the type defined in ``vehicle_class`` (a superclass of :py:class:`Vehicle`).
    """

    from dronekit.mavlink import MAVConnection

    if not vehicle_class:
        vehicle_class = Vehicle

    handler = MAVConnection(ip, baud=baud, source_system=source_system, source_component=source_component, use_native=use_native)
    vehicle = vehicle_class(handler)

    if status_printer:
        vehicle._autopilot_logger.addHandler(ErrprinterHandler(status_printer))

    if _initialize:
        vehicle.initialize(rate=rate, heartbeat_timeout=heartbeat_timeout)

    if wait_ready:
        if wait_ready is True:
            vehicle.wait_ready(still_waiting_interval=still_waiting_interval,
                               still_waiting_callback=still_waiting_callback,
                               timeout=timeout)
        else:
            vehicle.wait_ready(*wait_ready)

    return vehicle
