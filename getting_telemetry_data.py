                           # This code has been written by Serkan Butun to test how telemetry commands work.

import sys, time , math, datetime 	
from pymavlink import mavutil,mavwp


# create connection
#master = mavutil.mavlink_connection("/dev/serial0", baud=57600) #The connection to be used when establishing a serial connection.
#master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200, wait_ready=True) #The connection to be used when establishing a serial connection.
master = mavutil.mavlink_connection('0.0.0.0:14551',wait_ready=True) #The connection to be used when establishing a connection with simulation.
master.wait_heartbeat()
print('connected')

master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 2, 1)

def telemetry():
    msg = master.recv_match(blocking=True)
    if msg.name == 'GLOBAL_POSITION_INT':
        UAV_altitude = msg.relative_alt/1000
        print(" UAV_altitude = ", UAV_altitude)
    
    
    msg = master.recv_match(blocking=True)
    if msg.name == 'ATTITUDE':
        UAV_pitch = math.degrees(msg.pitch)
        UAV_yaw = math.degrees(msg.yaw)
        UAV_roll = math.degrees(msg.roll)
        print(" UAV_pitch = ", UAV_pitch)
        print(" UAV_yaw = ", UAV_yaw)
        print(" UAV_roll = ", UAV_roll)
    
    
    msg = master.recv_match(blocking=True)
    if msg.name == 'GPS_RAW_INT':
        print(datetime.datetime.fromtimestamp(msg.time_usec / 1000.0, tz=datetime.timezone.utc))
        print((datetime.datetime.fromtimestamp(msg.time_usec / 1000.0, tz=datetime.timezone.utc).strftime('%H:%M:%S.%f')))
        UAV_latitude = msg.lat / 1.0e7
        UAV_longtitude = msg.lon / 1.0e7
        UAV_velocity = msg.vel/100
        print(" UAV_latitude = ", UAV_latitude)
        print(" UAV_longtitude = ", UAV_longtitude)
        print(" UAV_velocity = ", UAV_velocity)
    
    
    msg = master.recv_match(blocking=True)
    if msg.name == 'BATTERY_STATUS':
        UAV_battery = msg.battery_remaining
        print(" UAV_battery = ", UAV_battery)


while True:
    telemetry()
