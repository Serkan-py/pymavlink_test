                     # This code has been written by Serkan Butun to test how the velocity commands work.

import time
from pymavlink import mavutil


#master = mavutil.mavlink_connection("/dev/serial0", baud=57600)
master = mavutil.mavlink_connection('127.0.0.1:14550',wait_ready=True)
master.wait_heartbeat()
print('connected')


mode = 'GUIDED'
master.set_mode('GUIDED')
print(mode)


#arming
master.arducopter_arm()
time.sleep(1)


target_altitude = 3

master.mav.command_long_send(
1, #1# autopilot system id.
1, #1# autopilot component id
22, # command id, TAKEOFF
0, # confirmation
0.0,0.0,
0.0,0.0,0.0,0.0,target_altitude, # unused parameters for this command,
force_mavlink1=False
)


msg = master.recv_match(type="GLOBAL_POSITION_INT",blocking=True)
altitude = msg.relative_alt/1000
print(altitude)


#It checks whether the target altitude has been reached or not.
while altitude <= target_altitude*0.96:
	msg = master.recv_match(type="GLOBAL_POSITION_INT",blocking=True)
	altitude = msg.relative_alt/1000
	print(altitude)


#master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, 1, 3527 , 0, 0, 0, 0.2, 0.0, 0.0, 0, 0, 0, 0 ,0)
#master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, 1, 3576 , 1, 0, -3, 0.0, 0.0, 0.0, 0, 0, 0, 0 ,0)

#Velocity adjustment.
velocity = 0.5
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, velocity, 0, 0, 0, 0, 0)


#If the movement point is set to 9, the drone will move instantaneously according to its own body.
#If the movement point is set to 1, the drone will consider its armed direction and move accordingly.

movement point = 1
masking = 3576#3527 (m/s), 3576 (m)
t_altitude = -3#-yukari yon oluyor

time.sleep(2)
print(' Forward orientation begins.')
master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, movement point, masking , 1, 0, t_altitude, 0.0, 0.0, 0.0, 0, 0, 0, 0 ,0)


time.sleep(4)
print(' Reverse orientation begins.')
master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, movement point, masking , 0, 0, t_altitude, 0.0, 0.0, 0, 0, 0, 0, 0 ,0)


time.sleep(4)
print(' Rightward orientation begins.')
master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, movement point, masking , 0, 1, t_altitude, 0.0, 0.0, 0.0, 0, 0, 0, 0 ,0)


time.sleep(4)
print(' Lefttward orientation begins.')
master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, movement point, masking , 0, 0, t_altitude, 0, 0.0, 0, 0, 0, 0, 0 ,0)


time.sleep(2)
print(' Forward and rightward orientation begins.')
master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, movement point, masking , 1, 1, t_altitude, 0.0, 0.0, 0.0, 0, 0, 0, 0 ,0)


time.sleep(2)
print(' Forward and leftward orientation begins.')
master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, movement point, masking , 2, 0, t_altitude, 0.0, 0.0, 0.0, 0, 0, 0, 0 ,0)


time.sleep(4)
print(' Reverse and rightward orientation begins.')
master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, movement point, masking , 1, 1, t_altitude, 0.0, 0.0, 0, 0, 0, 0, 0 ,0)



time.sleep(4)
print(' Reverse and leftward orientation begins.')
master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, movement point, masking , 0, 0, t_altitude, 0.0, 0.0, 0, 0, 0, 0, 0 ,0)

