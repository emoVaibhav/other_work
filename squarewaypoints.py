from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse  

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

 
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) 

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)

        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
#MAIN
arm_and_takeoff(10)
target_location = LocationGlobalRelative(28.752859412269224, 77.1157724859942, 30)
vehicle.simple_goto(target_location)
time.sleep(15)
target_location = LocationGlobalRelative(28.753444711599567, 77.11577699691436, 30)
vehicle.simple_goto(target_location)
time.sleep(15)
target_location = LocationGlobalRelative(28.754049781107156, 77.11671977922846, 30)
vehicle.simple_goto(target_location)
time.sleep(15)

target_location = LocationGlobalRelative(28.75292664276449, 77.1165190432812, 30)
vehicle.simple_goto(target_location)

time.sleep(15)

print("Now let's land")
vehicle.mode = VehicleMode("RTL")

vehicle.close()
