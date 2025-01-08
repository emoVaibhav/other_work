from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

# Connect to the vehicle (in this case SITL)
vehicle = connect('127.0.0.1:14550', wait_ready=False)


def send_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # Control vx, vy, vz
        0, 0, 0,             # x, y, z positions (not used)
        vx, vy, vz,          # velocity in m/s
        0, 0, 0,             # acceleration (not supported)
        0, 0)               
    vehicle.send_mavlink(msg)
    vehicle.flush()

def arm_and_takeoff(target_altitude):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)
    
  
    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {altitude}")
        if altitude >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def fly_circle(radius, velocity, duration):
    i=0
    while i<=360:
     
        vx = velocity * math.cos(math.radians(i))
        vy = velocity * math.sin(math.radians(i))
        i=i+1
        send_velocity(vx, vy, 0)
        print('sent velocity',vx,vy)
        time.sleep(0.5)

# Main code
try:
    arm_and_takeoff(20) 

    print("Starting circular movement")
    radius = 3
    velocity = 5 
    duration = 10

    fly_circle(radius, velocity, duration)

    print("Landing")
    vehicle.mode = VehicleMode("LAND")

finally:
   
    vehicle.close()
    print("Vehicle connection closed")
