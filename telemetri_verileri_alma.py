# BU KOD TELEMETRI KOMUTLARININ NASIL CALISTIGINI TEST ETMEK AMACIYLA SERKAN BUTUN TARAFINDAN YAZILMISTIR 


import sys, time , math, datetime 	
from pymavlink import mavutil,mavwp


# baglanti olusturma kismi
#master = mavutil.mavlink_connection("/dev/serial0", baud=57600) #serial baglanti yapildiginda kullanilacak olan baglanti
#master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200, wait_ready=True) #serial baglanti yapildiginda kullanilacak olan baglanti
master = mavutil.mavlink_connection('0.0.0.0:14551',wait_ready=True) #simulasyon ile baglanti yapildiginda kullanilacak olan baglanti
master.wait_heartbeat()
print('baglandi')

master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 2, 1)

def telemetri():
    msg = master.recv_match(blocking=True)
    if msg.name == 'GLOBAL_POSITION_INT':
        IHA_irtifa = msg.relative_alt/1000
        print(" IHA_irtifa = ", IHA_irtifa)
    
    
    msg = master.recv_match(blocking=True)
    if msg.name == 'ATTITUDE':
        IHA_dikilme = math.degrees(msg.pitch)
        IHA_yonelme = math.degrees(msg.yaw)
        IHA_yatis = math.degrees(msg.roll)
        print(" IHA_dikilme = ", IHA_dikilme)
        print(" IHA_yonelme = ", IHA_yonelme)
        print(" IHA_yatis = ", IHA_yatis)
    
    
    msg = master.recv_match(blocking=True)
    if msg.name == 'GPS_RAW_INT':
        print(datetime.datetime.fromtimestamp(msg.time_usec / 1000.0, tz=datetime.timezone.utc))
        print((datetime.datetime.fromtimestamp(msg.time_usec / 1000.0, tz=datetime.timezone.utc).strftime('%H:%M:%S.%f')))
        IHA_enlem = msg.lat / 1.0e7
        IHA_boylam = msg.lon / 1.0e7
        IHA_hiz = msg.vel/100
        print(" IHA_enlem = ", IHA_enlem)
        print(" IHA_boylam = ", IHA_boylam)
        print(" IHA_hiz = ", IHA_hiz)
    
    
    msg = master.recv_match(blocking=True)
    if msg.name == 'BATTERY_STATUS':
        IHA_batarya = msg.battery_remaining
        print(" IHA_batarya = ", IHA_batarya)


while True:
    telemetri()


