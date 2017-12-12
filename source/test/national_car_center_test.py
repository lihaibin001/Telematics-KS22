import serial
import sys
import time
import socket


VIN = b'00000000F11111111'
ICCID = b'12345678901234567890'


def current_time():
    rt = [0, 0, 0, 0, 0, 0]
    current = time.localtime(time.time())
    rt[0] = current.tm_year % 100
    rt[1] = current.tm_mon
    rt[2] = current.tm_mday
    rt[3] = current.tm_hour
    rt[4] = current.tm_min
    rt[5] = current.tm_sec
    return bytearray(rt)


def login_data(time, log_id, iccid, sub_num, sub_len, sub_code):
    data = time\
           + log_id.to_bytes(2, byteorder='big')\
           + iccid \
           + sub_num.to_bytes(1, byteorder='big')\
           + sub_len.to_bytes(1, byteorder='big')\
           + sub_code
    return data


def logout_data(time, log_id):
    data = time + log_id.to_bytes(2, byteorder='big')
    return data


def vehicle_data(time, msg_id, msg_data):
    data = time + msg_id + msg_data
    return data


def car_body(status, charge, mode, speed, mileage, vol, current, soc, dc_dc, gear, res):
    data = status\
           + charge.to_bytes(1, byteorder='big')\
           + mode.to_bytes(1, byteorder='big')\
           + speed.to_bytes(2, byteorder='big')\
           + mileage.to_bytes(4, byteorder='big')\
           + vol.to_bytes(2, byteorder='big')\
           + current.to_bytes(2, byteorder='big')\
           + soc.to_bytes(1, byteorder='big')\
           + dc_dc.to_bytes(1, byteorder='big')\
           + gear+res.to_bytes(2, byteorder='big')\
           + b'00'
    return data


def motor_data(status, ctl_temp, rpm, torque, temp, ctl_vol, ctl_current):
    data = b'\x01\x01'\
         + status.to_bytes(1, byteorder='big')\
         + ctl_temp.to_bytes(1, byteorder='big')\
         + rpm.to_bytes(2, byteorder='big')\
         + torque.to_bytes(2, byteorder='big')\
         + temp.to_bytes(1, byteorder='big')\
         + ctl_vol.to_bytes(2, byteorder='big')\
         + ctl_current.to_bytes(2, byteorder='big')
    return data


# def engine_data():
#    data=
#    return data


# maximum ratings
# def MR_data():
#    data=
#    return data


# def alarm_data():
#    data=
#    return data


# cmd_id 1 byte; ack_flag 1 byte; vin 17 bytes; encrypt_flag 1 byte; len 2 bytes.
def ev_car_msg(cmd_id, ack_flag, vin, encrypt_flag, length, data):
    start_bytes = b'\x23\x23'
    if length > 0:
        data = start_bytes\
               + cmd_id\
               + ack_flag\
               + vin+encrypt_flag\
               + length.to_bytes(2, byteorder='big')\
               + data
    else:
        data = start_bytes\
               + cmd_id\
               + ack_flag\
               + vin+encrypt_flag\
               + length.to_bytes(2, byteorder='big')
    if ack_flag == b'\xFE':
        # command packet
        print("command sent")
    checksum = 0
    for d in data:
        checksum ^= d
    print(checksum)
    data += checksum.to_bytes(1, byteorder='big')
    print(data)
    return data

# ev_car_data=ev_car_msg()
login_id = 1
login_data_tmp = login_data(current_time(), login_id, ICCID, 1, 1, b'1')
logout_data_tmp = logout_data(current_time(), login_id)
car_body_data = car_body(b'\x03', 3, 1, 10, 100, 1000, 1200, 50, 1, b'\x3E', 500)
# Command id: login=1, realtime=2, backup=3, logout=4, platform_login=5, platform_logout=6
ev_car_heartbeat = ev_car_msg(b'\x07', b'\xFE', VIN, b'\x01', 0, 0)
ev_car_login = ev_car_msg(b'\x01', b'\xFE', VIN, b'\x01', len(login_data_tmp), login_data_tmp)
ev_car_logout = ev_car_msg(b'\x04', b'\xFE', VIN, b'\x01', len(logout_data_tmp), logout_data_tmp)
ev_car_realtime_data_1 = ev_car_msg(b'\x02', b'\xFE', VIN, b'\x01', len(car_body_data), car_body_data)
ev_car_backup_data_1 = ev_car_msg(b'\x03', b'\xFE', VIN, b'\x01', len(car_body_data), car_body_data)
host = "218.205.176.44"
port = 19006
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))
s.sendto(ev_car_heartbeat, (host, port))
print(s.recv(24))
s.sendto(ev_car_login, (host, port))
s.recv(24)
s.sendto(ev_car_realtime_data_1, (host, port))
s.recv(24)
s.sendto(ev_car_backup_data_1, (host, port))
s.recv(24)
s.sendto(ev_car_logout, (host, port))
s.recv(24)
s.close()
