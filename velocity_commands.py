# BU KOD VELOCITY KOMUTLARININ NASIL CALISTIGINI TEST ETMEK AMACIYLA SERKAN BUTUN TARAFINDAN YAZILMISTIR 


import time
from pymavlink import mavutil


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


hedef_yukseklik = 3

master.mav.command_long_send(
1, #1# autopilot system id.
1, #1# autopilot component id
22, # command id, TAKEOFF
0, # confirmation
0.0,0.0,
0.0,0.0,0.0,0.0,hedef_yukseklik, # unused parameters for this command,
force_mavlink1=False
)


msg = master.recv_match(type="GLOBAL_POSITION_INT",blocking=True)
altitude = msg.relative_alt/1000
print(altitude)


#hedef yukseklige geldi mi kontrol eder
while altitude <= hedef_yukseklik*0.96:
	msg = master.recv_match(type="GLOBAL_POSITION_INT",blocking=True)
	altitude = msg.relative_alt/1000
	print(altitude)


#master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, 1, 3527 , 0, 0, 0, 0.2, 0.0, 0.0, 0, 0, 0, 0 ,0)
#master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, 1, 3576 , 1, 0, -3, 0.0, 0.0, 0.0, 0, 0, 0, 0 ,0)

#Hiz ayarlama
hiz = 0.5
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, hiz, 0, 0, 0, 0, 0)


#haraket_noktasi 9 olursa dron anlik olarak kendi govdesine gore haraket eder
#haraket_noktasi 1 olursa dron arm olduğu yönü baz alır ve ona göre haraket eder

haraket_noktasi = 1
maskeleme = 3576#3527 (m/s), 3576 (m)
yukseklik = -3#-yukari yon oluyor

time.sleep(2)
print(' ileri yonelme basliyor')
master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, haraket_noktasi, maskeleme , 1, 0, yukseklik, 0.0, 0.0, 0.0, 0, 0, 0, 0 ,0)


time.sleep(4)
print(' geri yonelme basliyor')
master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, haraket_noktasi, maskeleme , 0, 0, yukseklik, 0.0, 0.0, 0, 0, 0, 0, 0 ,0)


time.sleep(4)
print(' saga yonelme basliyor')
master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, haraket_noktasi, maskeleme , 0, 1, yukseklik, 0.0, 0.0, 0.0, 0, 0, 0, 0 ,0)


time.sleep(4)
print(' sola yonelme basliyor')
master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, haraket_noktasi, maskeleme , 0, 0, yukseklik, 0, 0.0, 0, 0, 0, 0, 0 ,0)


time.sleep(2)
print(' ileri ve saga yonelme basliyor')
master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, haraket_noktasi, maskeleme , 1, 1, yukseklik, 0.0, 0.0, 0.0, 0, 0, 0, 0 ,0)


time.sleep(2)
print(' ileri ve sola yonelme basliyor')
master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, haraket_noktasi, maskeleme , 2, 0, yukseklik, 0.0, 0.0, 0.0, 0, 0, 0, 0 ,0)


time.sleep(4)
print(' geri ve saga yonelme basliyor')
master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, haraket_noktasi, maskeleme , 1, 1, yukseklik, 0.0, 0.0, 0, 0, 0, 0, 0 ,0)



time.sleep(4)
print(' geri sola yonelme basliyor')
master.mav.set_position_target_local_ned_send(master.target_system, master.target_component, 0, haraket_noktasi, maskeleme , 0, 0, yukseklik, 0.0, 0.0, 0, 0, 0, 0, 0 ,0)

