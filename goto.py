##########################################################
##  Some of the functions are taken from https://dronekit-python.
# readthedocs.io/en/latest/examples/guided-set-speed-yaw-demo.html
# #example-guided-mode-goto-position-target-local-ned
#
# which is the dronekit documentation
#
# Functions include: condition_yaw, goto_position_target_local_ned,
# condition_yaw

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil 
import time
import math
#import pyrealsense2 as rs
from subprocess import PIPE
import subprocess
import RPi.GPIO as GPIO


######################################################################################
#------------------------------- Rover Functions ------------------------------------#
######################################################################################

def change2_guided_mode():
    while not vehicle.mode == "GUIDED":
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)
    print(vehicle.mode)
    

def change2_hold_mode():
    while not vehicle.mode == "HOLD":
        vehicle.mode = VehicleMode("HOLD")
        time.sleep(1)
    print(vehicle.mode)


def arm_vehicle():
    vehicle.armed = True
    while not vehicle.armed: time.sleep(1)


def arm_and_takeoff():
    print("Arming motors--2 sec wait")

    arm_vehicle()
    print("Vehicle Armed: %s\n"%vehicle.armed)

    # once armable vehile set to GUIDED mode
    change2_guided_mode()
    print("--Bot Mode-- %s" %vehicle.mode)



def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


######################################################################################
#------------------------------- Mavlink/Rover Functions ----------------------------#
######################################################################################

def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration, duration_mult):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
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
        vehicle.send_mavlink(msg)
        ##TODO: change back to 1
        time.sleep(duration_mult)


def send_relative_velocity(velocity_x, velocity_y, velocity_z, duration, duration_mult):
    """
    This function is relative to the bot's position

    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(duration_mult)

def goto_position_target_local_ned(north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)

def goto_position_target_local_relative(north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)



def condition_yaw(heading, relative=False):
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
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


#---------------------------------- Ultra Sonic Sensor functions ------------------

def obstacle_from_ultra():
    print "Measuring..."
    GPIO.setmode(GPIO.BCM)

    TRIG = 23
    ECHO = 24

    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
    GPIO.output(TRIG, False)

    print("\nChecking for obstacle ultra...\n")
    us_dist_arr = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
    for i in range(20):
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

       # pulse_start = time.time()

        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()
        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        us_distance = pulse_duration * 17150
        us_distance = round(us_distance, 2)
        ##TODO: change distance to us_distance in other code

        us_dist_arr.pop(0)
        us_dist_arr.append(us_distance)

        j = 0
        for x in us_dist_arr:
            ##TODO: change back x > 20 and x<300, for testing
            if x > 50 and x < 400:
                j = j + 1

        if j >= 10:
            immediate_obj = 1
            print("\nObstacle at distance: %s"%us_dist_arr)
            print("\n--------------OBSTACLE DETECTED--------------\n")
            print("This is immediate_obj: %s"%immediate_obj)
            break

        else:
            immediate_obj = 0
            print("\n<<<<<<<<<<<<<<< no obstacle detected>>>>>>>>>>>>>>>\n")
    GPIO.cleanup()

    return immediate_obj


def get_velo_duration(x_coord, y_coord, gndspd, lidarDist):
    x_velo = (x_coord*gndspd)/lidarDist
    y_velo = (y_coord*gndspd)/lidarDist
    duration = lidarDist/gndspd
    return x_velo, y_velo, duration

######################################################################################
#------------------------------- Main Code ------------------------------------------#
######################################################################################


"""
#setting up tracking camera
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
#finish camera setup

pipe.start(cfg)
"""
gndspd = 1
connection_string = "/dev/ttyS0"
#Connect to the vehicle
print("Connection to the vehicle on %s"%connection_string)
vehicle = connect(connection_string,baud = 57600, wait_ready=False)

arm_and_takeoff()
x=0

vehicle.simple_goto(LocationGlobalRelative(30.6231131, -96.3451976,0))
print("In time.sleep")
time.sleep(10)
"""
home_loc = vehicle.location.global_frame
send_ned_velocity(-1,0,0,10,1)
for i in range(3):
    x_velo, y_velo, duration_mult = get_velo_duration(-5, -5, gndspd, 1)
    print("Duration: %s"%duration_mult)
    send_ned_velocity(x_velo,y_velo,0,1, duration_mult)
    send_relative_velocity(0,0,0,1, 1.5)
    #time.sleep(0)
    
send_relative_velocity(0,0,0,1, 1)

last_loc = vehicle.location.global_frame
print("Home loc: %s"%home_loc)
print("Last loc: %s"%last_loc)
distance = get_distance_metres(home_loc, last_loc)
print("distance: %s"%distance)


current_angle = 1
best_angle = 1

for i in range(10):
    time.sleep(1)
    print("======================This is iteration: %s====================="%i)
    try:

        print("Would be collecting lidar data...\n")
        print("sleeping for 5 second\n")
        time.sleep(5)

        best_angle = 0
        current_angle = 0

        if best_angle == current_angle:
            immediate_obj = 0
            while obstacle_from_ultra()==0:
                send_ned_velocity(-0.7, -0.7, 0, 1)
                print("Going until obstacle detected...\n")
        
    except KeyboardInterrupt:
        break

"""

change2_hold_mode()

vehicle.armed = False
    # if armed wait
while vehicle.armed:
    time.sleep(1)
    vehicle.armed = False

print("Vehicle Armed: %s" % vehicle.armed)
vehicle.close()
print("Vehicle Closed -- Mission Over")
time.sleep(5)



