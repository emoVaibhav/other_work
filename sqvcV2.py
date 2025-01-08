from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

# Connect to the vehicle (in this case SITL)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Function to send velocity commands to the drone
def send_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  
        0, 0, 0,             
        vx, vy, vz,          
        0, 0, 0,             
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
    # Wait until the vehicle reaches the target altitude
    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {altitude}")
        if altitude >= (int(target_altitude) * 0.95):
            print("Reached target altitude")
            break
        time.sleep(1)

def haversine(lat1, lon1, lat2, lon2):
     
    # distance between latitudes
    # and longitudes
    dLat = (lat2 - lat1) * math.pi / 180.0
    dLon = (lon2 - lon1) * math.pi / 180.0
 
    # convert to radians
    lat1 = (lat1) * math.pi / 180.0
    lat2 = (lat2) * math.pi / 180.0
 
    # apply formulae
    a = (pow(math.sin(dLat / 2), 2) +
         pow(math.sin(dLon / 2), 2) *
             math.cos(lat1) * math.cos(lat2));
    rad = 6371*1000
    c = 2 * math.asin(math.sqrt(a))
    return rad * c

def bearing(lat1, lon1, lat2, lon2): #(-180,180)
    
    dLon = math.radians(lon2 - lon1)

    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)

    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)

    bearing = math.degrees(math.atan2(y, x))
    print("this is ", bearing)

    return bearing


def PID_gen():
    kp,ki,kd = 0.43,0.001,0.8
    global velocity 
    global error_sum
    dt =0.01
    global error_old
    while True:
        lat1 = vehicle.location.global_frame.lat
        lon1= vehicle.location.global_frame.lon
        velx,vely,velz =vehicle.velocity
        #print('difference is' ,bearing(lat1, lon1,lat2,lon2)- math.degrees(math.atan2(velx,vely)))
        if bearing(lat1, lon1,lat2,lon2)- math.degrees(math.atan2(velx,vely)) < 90:
           const = -1
        else:
            const = 1
        error_new = haversine(lat1,lon1, lat2, lon2)
        error_sum = error_sum+error_new
        #print('error sum is',error_sum, 'error old', error_old , 'error new', error_new)
        integral = (error_sum)*ki*dt
        prop = kp*error_new
        vel = math.sqrt(velx**2+vely**2+velz**2)
        diff = const*vel*kd
        print( 'differential is', diff)
        velocity = integral+diff+prop
        #print('error old is', error_old,'error new is',error_new,'differential is',diff)

        error_old = error_new
        
       
        time.sleep(dt)
        yield velocity

def PID_vel():
    lat1 = vehicle.location.global_frame.lat
    lon1= vehicle.location.global_frame.lon
    while haversine(lat1,lon1,lat2,lon2)>0.05:
        lat1 = vehicle.location.global_frame.lat
        lon1 = vehicle.location.global_frame.lon
        pidcall = PID_gen()
        pidvel = next(pidcall)
        vx = pidvel*math.cos(math.radians(bearing(lat1,lon1, lat2, lon2)))
        vy = pidvel*math.sin(math.radians(bearing(lat1, lon1, lat2, lon2)))
        v_plot.append(velocity)
        e_plot.append(haversine(lat1 ,lon1,lat2,lon2))
        send_velocity(vx,vy,0)
        time.sleep(0.1)
        print(haversine(lat1,lon1,lat2,lon2))
        
#main
arm_and_takeoff(10)
v_plot = []
e_plot =[]
list_coord = [(28.75167034, 77.11607897),(28.75264946 , 77.11535329)]
error_old = 0
error_sum = 0
for i in list_coord:
    lat2 = i[0]
    lon2 =i[1]
    print("Going to point", i)
    PID_vel()
    send_velocity(0,0,0)
    time.sleep(5)
    

vehicle.mode = VehicleMode("RTL")


import matplotlib.pylab as plt

plt.plot(range(len(v_plot)), v_plot, marker='o', linestyle='-', color='b')
plt.plot(range(len(e_plot)), e_plot, marker='o', linestyle='-', color='g')
# Add labels and title
plt.xlabel('Index')
plt.ylabel('Value')
plt.title('Plot of List v with Index')
print (len(v_plot))
print(len(e_plot))

# Show the plot
plt.show()
