
from dronekit import connect, VehicleMode, LocationGlobalRelative
import serial
from xbee import XBee
import time
import math
from xbee.thread import XBee

#########################################################################
# this code uses the xbee for python 2 so that it can work with dronekit#
#########################################################################

connection_string = "/dev/ttyS0"
#-- Connect to the vehicle --
print("Connection to the vehicle on %s" % connection_string)
vehicle = connect(connection_string, baud=57600, wait_ready=False)
print("<<<<<<<<<<Connecting to Vehicle>>>>>>>>>")

#initialize the waypoints array for the follower -- will be populated with wps recieved by xbee
global waypoints
waypoints = []

 
#------------------------------------- Rover Functions-------------------------------------------#

def arm_vehicle():
    # arms the Pixhwk usign dronekit params
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
        vehicle.armed = True

def arm_and_takeoff():
    #prepares the Pixhawk for the mission -- arms and changes bot to guided mode for go to commands
    print("Arming motors -- 3 sec wait")
    
    arm_vehicle()
    change2_guided_mode()
    
    print("Vehicle Mode: %s" % vehicle.mode)
    print("Vehicle Armed: %s" % vehicle.armed)

def change2_hold_mode():
    # changes bot to HOLD mode which keeps bot motionless
    while not vehicle.mode == "HOLD":
        vehicle.mode = VehicleMode("HOLD")
        time.sleep(1)    
    print(vehicle.mode)
        
def change2_guided_mode():
    # changes bot to guided mode -- which allows bot to move given MavLink message commands
    while not vehicle.mode == "GUIDED":
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)
    print(vehicle.mode)


#--------------------------------------- XBee Functions-------------------------------------------#
def message_received(data):
    #XBee Function that runs right when data is received by XBee
    #prints the entire string received by xbee
    print(data)    
    #gets the rf data from the string -- this is the lat and lon sent from the leader
    rf_data = data["rf_data"]
    
    #updating the leader current location by adding the spliting the rf data into a list with lat and lon, then is appended to the waypoints list
    waypoint = rf_data.split()
    waypoints.append(waypoint)   
    
    return data

#--------------------------------------- Math Functions-------------------------------------------#

def time2sleep(current_loc, wp, gndspeed):
    #calculates time to sleep for a simple go to based on distance to point and vehicle speed
    dist = get_distance_metres(current_loc, wp)
    time = dist/gndspeed
    
    return time

def get_distance_metres(aLoc1, aLoc2):
    # finds a returns distance in meters from two Location Global Relative variables
    dlat = aLoc2.lat - aLoc1.lat
    dlong = aLoc2.lon - aLoc1.lon
    
    return math.sqrt((dlat*dlat)+(dlong*dlong))*1.113195e5

#------ MAIN PROGRAM ------------
#arming the autopilot and setting to guided mode
arm_and_takeoff()

#setting groundspeed to 1.5 m/s
gndspd = 1.5
vehicle.groundspeed = gndspd

# connecting to xbee device via USB connection
print("------------Trying to connect to xbee-------------")
#Setting up XBee port
PORT = "/dev/ttyUSB0"
xbee_port = serial.Serial(PORT, 9600)
xbee = XBee(xbee_port, callback=message_received)
print("------------Connected to xbee---------------")

#counter for which waypoint the bot is on
i = 0

#setting leader and follower home location to followers current location
f_home = vehicle.location.global_frame
l_loc = f_home
f_loc = f_home
dist_follow_lead = 0
moved2start = False
j = 0

#we run the while True until the bots arrive and the code breaks out -- or it ends with a safety except
while True:
    j=j+1
    # if we have not yet moved to leaders home location move -- only if we have recieved the home loc via xbee -- this if should only run once in the very beggining
    if moved2start == False and len(waypoints) > 1:

        #creating LocationGlobalRelative using lat and lon values from waypoints list
        l_lat = waypoints[0][0]
        l_lon = waypoints[0][1]
        l_lat = float(l_lat)
        l_lon = float(l_lon)
        l_loc = LocationGlobalRelative(l_lat,l_lon,0)

        #runs simple goto function which is a dronekit function that sends MavLink messages to Pixhawk to go to given coordinate location
        vehicle.simple_goto(l_loc)

        # we then have to calculate the time sleep for the go to command because if interrupted by another go to this one will stop and we must reach the destination before the next one is sent to avoid obstacles
        duration = time2sleep(f_loc, l_loc, gndspd)
        time.sleep(duration)
        moved2start = True

        #This while waits to make sure we get the next waypoint via xbee before continuing so we do not index out of list
        while not len(waypoints)>i+1:
            k = 0 
        #now we can iterate to the next point since we have received it
        i = i+1
        print("----------------iteration: %s-------------\n"%i)
        print("**Moved to Leaders Start**")

    try:
        if len(waypoints)>0:
            last_index = len(waypoints)-1
            
            # if leader has not arrived -- leaders current location is the last element in the waypoints list from xbee -- its data is converted into a LocationGlobalRelative to compare with the follwer loc
            #if end end is the last element of the list - leader has stopped going
            if not waypoints[last_index] == ["end", "end"]:
                l_lat = waypoints[last_index][0]
                l_lon = waypoints[last_index][1]
                l_lat = float(l_lat)
                l_lon = float(l_lon)
                l_loc = LocationGlobalRelative(l_lat,l_lon,0)

            # we can now compare the leader and follower current location using this function
            dist_follow_lead = get_distance_metres(f_loc, l_loc) 
            
            # check if distance between follower and lead is 7 meters
            if  dist_follow_lead > 7.0:
                print("Lead and Follow > 7m away \n")
                
                print("Lead Follow Dist: %s" %dist_follow_lead)

                #gets the x and y coords from the waypoint list
                wp_lat = float(waypoints[i][0])
                wp_lon = float(waypoints[i][1])
                
                #waits to get next waypoint via xbee before continuing so we do not index out of list
                while not len(waypoints)>i+1:
                    k=0
                #keeps track of which waypoint it is on in waypoint list
                i = i+1
                print("----------------iteration: %s-------------\n"%i)
                print("This is waypoint lat: %s"%wp_lat)
                print("This is waypoint lon: %s"%wp_lon)
                
                #creating wp for follower from wp list -- also finding how long to sleep while going to wp
                wp = LocationGlobalRelative(wp_lat, wp_lon, 0)
                duration = time2sleep(vehicle.location.global_frame, wp, gndspd)
                print("This is GO Duration: %s"%duration)
                
                #finds dist from follower to next wp
                f_loc = vehicle.location.global_frame
                f_loc2_wp = get_distance_metres(f_loc, wp)
                print("Followers distance to next wp: %s"%f_loc2_wp)

                #if the wp dist is > 1m we go to the waypoint
                if f_loc2_wp > 1:
                    vehicle.simple_goto(wp)
                    print(" /wpDist >1m -- waiting while going /n")
                    time.sleep(duration)
                #update the followers current location
                f_loc = vehicle.location.global_frame
            
            #this else is for when the bots are not 7m apart -- nothing should happen unless leader has arrived at final destination
            else:
                if len(waypoints)>0:
                    #this if is for the case when the leader has arrived to the destination and the follower is catching up
                    if waypoints[len(waypoints)-1] ==["end","end"]:
                        # this i+1 to not go to the last point 
                        if i+1 < len(waypoints) - 1:
                            #creating next wp for follower
                            wp_lat = float(waypoints[i][0])
                            wp_lon = float(waypoints[i][1])
                            print("inside special case close to end--leader has arrived \n")

                            #keeps track of which waypoint it is on in waypoint list
                            i = i+1
                            print("----------------iteration: %s-------------\n"%i)
                            print("This is waypoint lat: %s"%wp_lat)
                            print("This is waypoint long: %s"%wp_lon)
                            
                            #creating wp, finding drive duration, and going to wp
                            wp = LocationGlobalRelative(wp_lat, wp_lon, 0)
                            duration = time2sleep(vehicle.location.global_frame, wp, gndspd)
                            vehicle.simple_goto(wp)
                            time.sleep(duration)

                            print(">>>>>simple goto wp--leader already arrived")

                        else:
                            #This is the case where the follower is on the last wp from leader
                            #we do not go to the point -- to not hit leader
                            #checking lead and follow locs to compare

                            f_loc = vehicle.location.global_frame

                            wp_lat = float(waypoints[i-1][0])
                            wp_lon = float(waypoints[i-1][1])
                            
                            l_loc = LocationGlobalRelative(wp_lat, wp_lon, 0)
                            
                            leadFollowDist=get_distance_metres(l_loc,f_loc)
                            
                            print("\nin last case, break--dont go to final point")
                            print("Lead and Follow Distance Final: %s"%leadFollowDist)
                            # after this break the follower  bot has arrived one wp behind the leaders location 
                            break
    #this is a safety except -- a keyboard interrupt or if we get end end from the leader from a keyboard interrupt we will break the while true
    except KeyboardInterrupt or waypoints[i]==["end", "end"]:
        print("waypoints list: %s"%waypoints)
        break
    
print("*********Bot Arrived*************")

#closing rover, xbee, and ports
change2_hold_mode()
vehicle.armed = False
xbee.halt()
xbee_port.close()
vehicle.close()
time.sleep(2)






