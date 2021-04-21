
######################################################################################
##  Some of the functions are taken from https://dronekit-python.
# readthedocs.io/en/latest/examples/guided-set-speed-yaw-demo.html
# #example-guided-mode-goto-position-target-local-ned
# which is the dronekit documentation
#
# Functions include: condition_yaw, goto_position_target_local_ned, send_ned_velocity, 
# get_distance_metres
import RPi.GPIO as GPIO
import serial
from xbee import ZigBee
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil 
import time
import math
import argparse
import os
import re

#------------------------------- Connect to Rover -----------------------------------#

connection_string = "/dev/ttyS0"
print("Connection to the vehicle on %s"%connection_string)
vehicle = connect(connection_string,baud = 57600, wait_ready=False)

#------------------------------- Connect to XBee -----------------------------------#

serial_port = serial.Serial('/dev/ttyUSB1', 9600)
xbee = ZigBee(serial_port)

#------------------------------- Rover Functions ------------------------------------#

# Guided mode allows it to drive from go commands
def change2_guided_mode():
    while not vehicle.mode == "GUIDED":
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)
    print(vehicle.mode)
    
# Hold mode stops it, and interrupts a simple_goto
def change2_hold_mode():
    while not vehicle.mode == "HOLD":
        vehicle.mode = VehicleMode("HOLD")
        time.sleep(1)
    print(vehicle.mode)


def arm_vehicle():

    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
        vehicle.armed = True

def disarm_vehicle():
    vehicle.armed = False
    while vehicle.armed: time.sleep(1)

def arm_and_takeoff():
    print("Arming motors--2 sec wait")

    arm_vehicle()
    print("Vehicle Armed: %s\n"%vehicle.armed)

    # once armable vehile set to GUIDED mode
    change2_guided_mode()
    print("--Bot Mode-- %s" %vehicle.mode)


#------------------------------- Mavlink/Rover Functions ----------------------------#


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
        x = time.time()
        vehicle.send_mavlink(msg)
        #duration_mult sets the time to sleep per each duration, new distance = speed/(duration*duration_mult)
        time.sleep(duration_mult)
        timer = time.time() -x
        print("Time in send_ned: %s"%timer)


def send_relative_velocity(velocity_x, velocity_y, velocity_z, duration, duration_mult):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        #duration_mult sets the time to sleep per each duration, new distance = speed/(duration*duration_mult)
        time.sleep(duration_mult)




#------------------------------- Lidar Function ----------------------------#

def get_lidar_data():
    data = os.popen('sudo ./ultra_simple').read()
    print("This is the Lidar output:\n %s"%data)
    # time.sleep here to make sure the lidar finishes
    time.sleep(2)
    return data


#------------------------------- Angle Finding Functions ----------------------------#

def get_edge_angle(angle_range, current_angle, lidar_read_dist, heading):
    #Creates a buffer on the side of each edge for the bot to fit
    edge_buffer_degrees = 50
    
    if float(angle_range[0])==0.0 and float(angle_range[1])==359.0:
        print("\n<<<<<<< in case where it is 0 359 >>>>>>>>\n")
        return 0, current_angle
    #changes angle_range from lidar to cardinal degrees based on vehicles heading
    card_angle_range = lidar2cardinal_angle(angle_range, heading)

    if card_angle_range[0] > card_angle_range[1]:
        gap_size = ((360-card_angle_range[0])+card_angle_range[1])
    else:
        gap_size = abs(card_angle_range[1]-card_angle_range[0])

    # If the gap size is too small for rover
    if gap_size < edge_buffer_degrees:
        return 2000, 0

    # if the gap size is big enough for rover but too small for buffer, return midpoint
    if gap_size < 3*edge_buffer_degrees:
        midpoint_angle = ((gap_size/2) + card_angle_range[0])%360
        #checks if angle is greater than 180 to make sure its the real distance
        if abs(current_angle - midpoint_angle) > 180:
            dist1 = 360 - current_angle + midpoint_angle
            dist2 = 360 - midpoint_angle + current_angle
            edge_dist = min(dist1, dist2)
        else:
            edge_dist = abs(current_angle - midpoint_angle)

        return edge_dist, midpoint_angle

    else:
        #creates new angle range with buffer for obstacle
        card_angle_range[0] = card_angle_range[0] + edge_buffer_degrees
        card_angle_range[1] = card_angle_range[1] - edge_buffer_degrees

        #checks if angle is greater than 180 to make sure its the real distance
        if abs(current_angle - card_angle_range[0]) > 180:
            dist1 = 360 - current_angle + card_angle_range[0]
            dist2 = 360 - card_angle_range[0] + current_angle
            edge_dist1 = min(dist1, dist2)
        else:
            edge_dist1 = abs(current_angle - card_angle_range[0])


        #checks if angle is greater than 180 to make sure its the real distance
        if abs(current_angle - card_angle_range[1]) > 180:
            dist1 = 360 - current_angle + card_angle_range[1]
            dist2 = 360 - card_angle_range[1] + current_angle
            edge_dist2 = min(dist1, dist2)
        else:
            edge_dist2 = abs(current_angle - card_angle_range[1])


    # checks if the final angle is in one of the gaps
    if card_angle_range[0] > card_angle_range[1]:
        print("card angle range 0: %s" %card_angle_range[0])
        print("card angle range 1: %s" %card_angle_range[1])
        if current_angle>card_angle_range[0] or current_angle<card_angle_range[1]:
            print("(((In GAP)))-----weird case")
            return 0, current_angle

    if current_angle>card_angle_range[0] and current_angle<card_angle_range[1]:
        print("(((In GAP)))")
        return 0, current_angle


    if edge_dist1>edge_dist2:
        return edge_dist2, card_angle_range[1]
    else:
        return edge_dist1, card_angle_range[0]


def order_angles(gap_list, current_angle, lidar_read_dist, heading):
    angle_list = []
    edge_dist_list = []
    orderedAngles = []
    hold_dist_list = []
    for i in range(len(gap_list)):
        # here we get the best angle for the individual given range 
        edge_dist, angle = get_edge_angle(gap_list[i], current_angle, lidar_read_dist, heading)

        if not edge_dist == 2000:
            edge_dist_list.append(edge_dist)
            angle_list.append(angle)
    print("This is edge dist list: %s" %edge_dist_list)
    print("This is angle list: %s" %angle_list)
    for i in range(len(edge_dist_list)):
        hold_dist_list.append(edge_dist_list[i])
    
    #creates a list of ordered waypoints based on distance
    for i in range(len(angle_list)):
        #every next smallest distance
        minm = min(hold_dist_list)
        #this is the index of the minimum in the original list of distances
        index = edge_dist_list.index(minm)
        #this is the index of the minimum in the reduced list of distances
        index2 = hold_dist_list.index(minm)
        #Gets rid of the next smallest distance
        hold_dist_list.pop(index2)
        #appending the next best waypoint to order the waypoints
        orderedAngles.append(angle_list[index])
    
    #the minimum of the distance list is the closest to the final destination
    print("Ordered Angle List: %s \n"%orderedAngles)  

    return orderedAngles
# If there is an obstacle, finds the edge of the gap closest to the final angle -- ASSUMES lidar reads 5 m

def lidar2cardinal_angle(lidar_angle_range, heading):
    card_angle = [0,0]
    card_angle[0] = (int(float(lidar_angle_range[0])) + heading)%360
    card_angle[1] = (int(float(lidar_angle_range[1])) + heading)%360
    print("Cardinal Angle Gap: %s" %card_angle)
    return card_angle

#------------------------------- Math Functions ----------------------------#

def polar2cart(dist, angle):
    #angle in radians
    dist = float(dist)
    angle = float(angle)
    ang_rad = angle*(math.pi)/180
    x = dist*math.cos(ang_rad)
    y = dist*math.sin(ang_rad)

    return x,y

def cart2polar(current_x, current_y, final_x, final_y):
    #angle in radians
    x = float(final_x - current_x)
    y = float(final_y - current_y)
    dist = math.sqrt(x**2+y**2)
    angle_rad = math.atan(y/x)
    if (x < 0 and y < 0) or (x < 0 and y > 0):
        angle_rad = angle_rad + math.pi
    print(angle_rad)
    angle_degree = (angle_rad*180/(math.pi))%360

    return dist, angle_degree

def compare_dist(x1, y1, x2, y2):
    #x2,y2 should be the final coordinates
    #x1,y1 should be the waypoint to compare (or current location)
    y_diff = y2-y1
    x_diff = x2-x1
    dist = math.sqrt((y_diff**2) + (x_diff**2)) 
    return dist

def get_velo_duration(x_coord, y_coord, gndspd, lidarDist):
    x_velo = (x_coord*gndspd)/lidarDist
    y_velo = (y_coord*gndspd)/lidarDist
    duration = lidarDist/gndspd
    return x_velo, y_velo, duration


def get_angle(location1, location2):
    
    """ This function is from https://stackoverflow.com/questions/3932502/calculate-angle-between-two-latitude-longitude-points"""

    lat1 = location1.lat
    long1 = location1.lon
    
    lat2 = location2.lat
    long2 = location2.lon

    dLon = (long2 - long1)

    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1)* math.cos(lat2) * math.cos(dLon)

    brng = math.atan2(y, x)

    brng = brng * 180/math.pi
    brng = (brng + 360) % 360
    #brng = 360 - brng
    # count degrees counter-clockwise - remove to make clockwise

    return brng

def get_distance_metres(aLocation1, aLocation2):

    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


#---------------------------------- Ultra Sonic Sensor functions ------------------#

def obstacle_from_ultra():
    print( "Measuring...")
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

        pulse_start = time.time()

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
            #2 and 400 are the range in cm that the ultrasonic considers obstacles
            if x > 2 and x < 600:
                j = j + 1

        if j >= 14:
            immediate_obj = 1
            print("\nObstacle at distance: %s"%us_dist_arr)
            print("\n--------------OBSTACLE DETECTED--------------\n")
            print("This is immediate_obj: %s"%immediate_obj)
            send_relative_velocity(0,0,0,1,1)
            break

        else:
            immediate_obj = 0
            print("\n<<<<<<<<<<<<<<< no obstacle detected>>>>>>>>>>>>>>>\n")
    GPIO.cleanup()

    return immediate_obj

######################################################################################
#------------------------------- Main Code ------------------------------------------#
######################################################################################

#------------------------------- Get final location -----------------------------------#

# Gets final distance and angle from command line
parser = argparse.ArgumentParser(description='Sets Vehicles final destination')
#Gets the file from the command line
parser.add_argument('--file',help="File")

args = parser.parse_args()

file_name = str(args.file)

# k counts what in the files it is on
k=0

file = open(file_name, 'r')

for line in file:

    print("This is line in file %s"%line)
    print("############################ In waypoint: %s ##############################"%k)
    line=line.split(" ")

    #The file will have lattitude and longitudes of waypoints
    final_lat = float(line[0])
    final_lon = float(line[1])

    final_loc = LocationGlobal(final_lat, final_lon,0)
    home_GPS_location = vehicle.location.global_frame
    
    
    #sending follower leads home location - only on first waypoint
    if k ==0:
        data_for_follower = str(home_GPS_location.lat) + " " + str(home_GPS_location.lon)
        xbee.tx(dest_addr="\xFF\xFF", data=data_for_follower)
        print("\n Sending to follower: %s" %data_for_follower)

    k = k+1

    #final distance and angle to the current waypoint
    # distance in meters, angle in degrees
    final_dist = get_distance_metres(home_GPS_location, final_loc)
    final_angle = get_angle(home_GPS_location, final_loc)
    print("final angle = %s"%final_angle)
    print("final dist = %s"%final_dist)

    current_angle = final_angle%360
    print(current_angle)

    arm_and_takeoff()

    #lidar dist in m - distance it travels to get around an obstacle
    lidar_read_dist = 3

    #groundspeed in m/s
    gndspd = 1
    vehicle.groundspeed = gndspd

    # setting final dist and final x & y relative coords
    dist_to_final = final_dist
    final_x, final_y = polar2cart(final_dist, current_angle)

    #starts with the current location as 0,0
    current_x = 0
    current_y = 0

    print("---------------------START-=-------------------------")
    print("Final Relative x: %s" % final_x)
    print("Final Relative y: %s\n" % final_y)

    print("Current Relative x: %s" % current_x)
    print("Current Relative y: %s\n" % current_y)

    i = 0

    # while True will run until it is < 5 meters from the current waypoint
    # then it will break out
    while True:
        try:

            #CHECK IF CLOSE TO FINAL
            if dist_to_final ==0:
                print("\n<<<<<<<<<<<<<<<<<<<BOT ARRIVED>>>>>>>>>>>>>>>>\n")
                break
            elif dist_to_final < 5:
                print("\nBots final distance to point: %s\n"%dist_to_final)
                print("\n<<<<<<<<<<<<<<<<<<<BOT ARRIVED in < 5>>>>>>>>>>>>>>>>\n")
                break

            print("Waiting for Lidar data...")
            lidar_data = get_lidar_data()
            
            # Match uses regex to determine if the output of the C++ lidar code printed two angles
            # if it didn't, it printed a health code or something which we don't care about
            match = re.findall("\n(?P<angle1>[\d.]+(?= )) (?P<angle2>[\d.]+(?= ))", lidar_data.decode('utf-8'))

            print("THIS IS MATCH: %s"%match)        
            if match:

                # Finds polar to final from current position 
                current_dist, current_angle = cart2polar(current_x, current_y, final_x, final_y)

                heading = vehicle.heading
                print("Current Vehicle Heading:||| %s ||| \n"% heading)

                # sets distance to point = lidar dist or dist to final if < lidar dist
                needed_dist = lidar_read_dist

                print("\n ------current angle: %s --------------"%current_angle)

                # ordered_angles_list orders the angle options returned by the lidar from best to worst
                # It runs many other functions in it that find the best angle within the gaps from the lidar
                ordered_angle_list = order_angles(match, current_angle, needed_dist, heading)

                if len(ordered_angle_list)>0:

                    best_angle=ordered_angle_list[0]

                    #ULTRASONIC code - go to final unless obstacle detected
                    #When best_angle equals current_angle it means it is not trying to get around an obstacle 
                    #current_angle is the angle to the final destination at that point in time
                    if best_angle == current_angle:

                        # Runs velocity vector go first using the lidar data because it knows there
                        # will not be an obstacle there for as far as the lidar can see, so it does not
                        # need to run the ultrasonic
                        print("Doing first lidar before ultrasonic\n")
                        next_x, next_y = polar2cart(needed_dist, best_angle)
                        #finds the velocity vectors needed for the send_ned_velocity mavlink function
                        x_velo, y_velo, duration = get_velo_duration(next_x, next_y, gndspd, needed_dist)
                        print("Best angle = %s"%best_angle)

                        #solves for the gps location before going to the waypoint
                        prev_loc = vehicle.location.global_frame 
                        #this is the first lidar go to using velocity vectors
                        send_ned_velocity(x_velo, y_velo, 0 , duration, 1)
                        
                        # solving for the currentt x and y positions based on GPS data - found to be more accurate 
                        # than the velocity vectors expected position
                        gps_loc = vehicle.location.global_frame 

                        # Now we find next_x and next_y which are vectors we just travelled in order to calculate the new current_x and current_y
                        gps_dist = get_distance_metres(prev_loc,gps_loc)
                        gps_angle = get_angle(prev_loc, gps_loc)
                        next_x, next_y = polar2cart(gps_dist, gps_angle)

                        # Sends follower the leader's current GPS position
                        data_for_follower = str(gps_loc.lat) + " " + str(gps_loc.lon)
                        xbee.tx(dest_addr="\xFF\xFF", data=data_for_follower)
                        print("\n Sending to follower: %s" %data_for_follower)

                        current_x = current_x + next_x
                        current_y = current_y + next_y
                        print("Current Relative x: %s" % current_x)
                        print("Current Relative y: %s\n" % current_y)

                        # finds distance to the final location from current location 
                        dist_to_final = compare_dist(current_x, current_y, final_x, final_y)
                        print("***Distance to Final Loc: %s***\n" %dist_to_final)
                        
                        immediate_obj = 0
                        
                        prev_loc = gps_loc

                        #h is a counter to send the location to the follower every 3 times (not everytime)
                        h=0
                        while obstacle_from_ultra()==0:
                            
                            dist_to_final = get_distance_metres(gps_loc, final_loc)

                            if dist_to_final > 5:

                                prev_loc = gps_loc
                                
                                gps_loc = vehicle.location.global_frame

                                gps_dist = get_distance_metres(prev_loc,gps_loc)
                                gps_angle = get_angle(prev_loc, gps_loc)
                                next_x, next_y = polar2cart(gps_dist, gps_angle)

                                current_x = current_x + next_x
                                current_y = current_y + next_y
                            
                                #print("Would be sending follower but h = %s"%h)
                                if h%3 == 0:

                                    #Made a change here - send follower gps instead of x and y coords
                                    data_for_follower = str(gps_loc.lat) + " "+ str(gps_loc.lon)
                                    xbee.tx(dest_addr="\xFF\xFF", data=data_for_follower)
                                    print("\n Sending to follower: %s" %data_for_follower)

                                print("\n<<<<<<<<<<<<<<< no obstacle detected>>>>>>>>>>>>>>>\n")

                                h=h+1

                                vehicle.simple_goto(final_loc)
                                #vehicle.simple_goto(LocationGlobalRelative(30.6229403, -96.3451996,0))

                            else:
                                break

                        gps_loc = vehicle.location.global_frame 

                        gps_dist = get_distance_metres(prev_loc,gps_loc)
                        gps_angle = get_angle(prev_loc, gps_loc)
                        next_x, next_y = polar2cart(gps_dist, gps_angle)

                        current_x = current_x + next_x
                        current_y = current_y + next_y
                        print("Current Relative x: %s" % current_x)
                        print("Current Relative y: %s\n" % current_y)
                        
                        # when it stops for an obstacle, don't send that location

                        print("\nNot sending this point to follower\n")
                        change2_hold_mode()
                        change2_guided_mode()
                        

                         
                    #If there is an obstacle - goto lidar best angle 
                    else:
                        print("Best Angle: %s\n"%best_angle)
                        
                        # gets next x and y coord relative to current pos using best angle from above
                        next_x, next_y = polar2cart(needed_dist, best_angle)
                        print("Next x: %s" % next_x)
                        print("Next y: %s \n" % next_y)
                    
                        # Gets Velocitys (in X (+N,-S) and Y(+E,-W)) and Duration, using x and y coords and gndspeed
                        x_velo, y_velo, duration = get_velo_duration(next_x, next_y, gndspd, needed_dist)
                        print("Next x velo: %s" % x_velo)
                        print("Next y velo: %s" % y_velo)
                        print("Next duration: %s \n" % duration)

                        print("<Going to WP>")
                        prev_loc = vehicle.location.global_frame 
                        send_ned_velocity(x_velo, y_velo, 0 , duration, 1)

                        print("<Arrived at next lidar WP>\n")

                        gps_loc = vehicle.location.global_frame
        
                        gps_dist = get_distance_metres(prev_loc,gps_loc)
                        gps_angle = get_angle(prev_loc, gps_loc)
                        # solving for the correct x and y based on GPS data
                        next_x, next_y = polar2cart(gps_dist, gps_angle)

                        data_for_follower = str(gps_loc.lat) + " "+ str(gps_loc.lon)
                        xbee.tx(dest_addr="\xFF\xFF", data=data_for_follower)
                        print("\n Sending to follower: %s" %data_for_follower)

                        # Keeping track of total relative location (in x y plane) to where we started 
                        current_x = current_x + next_x
                        current_y = current_y + next_y
                        print("Current Relative x: %s" % current_x)
                        print("Current Relative y: %s\n" % current_y)

                        # finds distance to point from current location 
                        dist_to_final = compare_dist(current_x, current_y, final_x, final_y)
                        print("***Distance to Final Loc: %s***\n" %dist_to_final)
                        
                        i = i+1

        # Safety feature: we use a try and except so that we can KeyboardInterrupt if something goes wrong, and break out of the while loop       
        except KeyboardInterrupt or dist_to_final < 5:
            print("final_loc: %s"%final_loc)
            print("distance to final: %s"%get_distance_metres(vehicle.location.global_frame, final_loc))
            print("Current location x = %s"%current_x)
            print("Current location y = %s"%current_y)
            print("\n<<<<<<<<<<<<<<<<<BOT ARRIVED - in except>>>>>>>>>>>>>>")
            break


data_for_follower = "end end"
xbee.tx(dest_addr="\xFF\xFF", data=data_for_follower)
print("Sending follower end end")

#printing final GPS coordinate
gps_loc = vehicle.location.global_frame
print("Final GPS loc: %s"%gps_loc)

# change to HOLD mode and disarm
change2_hold_mode()
disarm_vehicle()

# Print and Close the Vehicle
print("Vehicle Armed: %s" % vehicle.armed)
vehicle.close()
print("Vehicle Closed -- Mission Over")
time.sleep(5)

