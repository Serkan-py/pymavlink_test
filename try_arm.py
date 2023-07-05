import time
from pymavlink import mavutil

# SADECE ARM MODU ÇALISIR

#master = mavutil.mavlink_connection("/dev/serial0", baud=57600)
master = mavutil.mavlink_connection('127.0.0.1:14550',wait_ready=True)

master.wait_heartbeat()
print('baglandi')

mode = 'GUIDED'
master.set_mode('GUIDED')
print(mode)


#arm eder
master.arducopter_arm()
time.sleep(1)
