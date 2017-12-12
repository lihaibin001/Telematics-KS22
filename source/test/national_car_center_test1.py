import serial
import sys
import time
import socket
import binascii


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


def login_data(login_time, log_id, iccid, sub_num, sub_len, sub_code):
    data = login_time \
           + log_id.to_bytes(2, byteorder='big') \
           + iccid \
           + sub_num.to_bytes(1, byteorder='big') \
           + sub_len.to_bytes(1, byteorder='big') \
           + sub_code
    return data


def logout_data(logout_time, log_id):
    data = logout_time + log_id.to_bytes(2, byteorder='big')
    return data


def vehicle_data(vehicle_time, msg_id, msg_data):
    data = vehicle_time + msg_id + msg_data
    return data


def car_body(status, charge, mode, speed, mileage, vol, current, soc, dc_dc, gear, res):
    data = b'\x01'\
           + status.to_bytes(1, byteorder='big')\
           + charge.to_bytes(1, byteorder='big')\
           + mode.to_bytes(1, byteorder='big')\
           + speed.to_bytes(2, byteorder='big')\
           + mileage.to_bytes(4, byteorder='big')\
           + vol.to_bytes(2, byteorder='big')\
           + current.to_bytes(2, byteorder='big')\
           + soc.to_bytes(1, byteorder='big')\
           + dc_dc.to_bytes(1, byteorder='big')\
           + gear.to_bytes(1, byteorder='big')\
           + res.to_bytes(2, byteorder='big')\
           + b'\x00\x00'
    return data


# driver motor assembly data
def driver_motor_assmbly_data(num, status, controller_temperature, speed, torque, temperature, voltage, current):
    data = num.to_bytes(1, byteorder='big')  \
           + status.to_bytes(1, byteorder='big') \
           + controller_temperature.to_bytes(1, byteorder='big') \
           + speed.to_bytes(2, byteorder='big') \
           + torque.to_bytes(2, byteorder='big') \
           + temperature.to_bytes(1, byteorder='big') \
           + voltage.to_bytes(2, byteorder='big') \
           + current.to_bytes(2, byteorder='big')
    return data


# driver motor data
def driver_motor_data(motor_amount, motor_mesg):
    data = b'\x02' + motor_amount.to_bytes(1, byteorder='big') + motor_mesg
    return data


# location data
def location_data(loc_status, longitude, latitude):
    data = b'\x05' \
           + loc_status.to_bytes(1, byteorder='big') \
           + longitude.to_bytes(4, byteorder='big') \
           + latitude.to_bytes(4, byteorder='big')
    return data


# extremum data
def extremum_data(max_vol_sub_sys_num, max_vol_bat_num,
                  bat_vol_max_val, min_vol_sub_sys_num,
                  min_vol_bat_num, bat_vol_min_val,
                  max_tmp_sub_sys_num, max_tmp_probe_num, max_tmp,
                  min_tmp_sub_sys_num, min_tmp_probe_num, min_tmp):
    data = b'\x06' \
           + max_vol_sub_sys_num.to_bytes(1, byteorder='big') \
           + max_vol_bat_num.to_bytes(1, byteorder='big') \
           + bat_vol_max_val.to_bytes(2, byteorder='big')  \
           + min_vol_sub_sys_num.to_bytes(1, byteorder='big') \
           + min_vol_bat_num.to_bytes(1, byteorder='big') \
           + bat_vol_min_val.to_bytes(2, byteorder='big') \
           + max_tmp_sub_sys_num.to_bytes(1, byteorder='big') \
           + max_tmp_probe_num.to_bytes(1, byteorder='big') \
           + max_tmp.to_bytes(1, byteorder='big') \
           + min_tmp_sub_sys_num.to_bytes(1, byteorder='big') \
           + min_tmp_probe_num.to_bytes(1, byteorder='big') \
           + min_tmp.to_bytes(1, byteorder='big')
    return data


# warning data
def warning_data(max_warning_level, normal_warning_level,
                 enegy_store_device_amount, enegy_store_device_fault_list,
                 drive_motor_fault_amount, drive_motor_fault_list,
                 motor_fault_amount, motor_fault_list,
                 other_fault_amount, other_falut_list):
    data = b'\x07' \
           + max_warning_level.to_bytes(1, byteorder='big') \
           + normal_warning_level.to_bytes(1, byteorder='big') \
           + enegy_store_device_amount.to_bytes(4, byteorder='big') \
           + enegy_store_device_fault_list.to_bytes(enegy_store_device_amount * 4, byteorder='big') \
           + drive_motor_fault_amount.to_bytes(1, byteorder='big') \
           + drive_motor_fault_list.to_bytes(4 * drive_motor_fault_amount, byteorder='big') \
           + motor_fault_amount.to_bytes(1, byteorder='big') \
           + motor_fault_list.to_bytes(4 * motor_fault_amount, byteorder='big') \
           + other_fault_amount.to_bytes(1, byteorder='big') \
           + other_falut_list.to_bytes(4 * other_fault_amount, byteorder='big')
    return data


# motor data
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


def enegy_device_info(sub_sys_num, vol, cur, bat_num, start_num, frame_bat_num, bat_vol):
    data = sub_sys_num.to_bytes(1, byteorder='big') \
           + vol.to_bytes(2, byteorder='big') \
           + cur.to_bytes(2, byteorder='big') \
           + bat_num.to_bytes(2, byteorder='big') \
           + start_num.to_bytes(2, byteorder='big') \
           + frame_bat_num.to_bytes(1, byteorder='big') \
           + bat_vol
    return data


def enegy_device_data(enegy_device_num, device_list):
    data = b'\x08' + enegy_device_num.to_bytes(1, byteorder='big') + device_list
    return data


def enegy_device_tmp_info(num, probe_num, tmp):
    data = num.to_bytes(1, byteorder='big') + probe_num.to_bytes(2, byteorder='big') + tmp
    return data


def enegy_device_tmp(num, device_list):
    data = b'\x09' + num.to_bytes(1, byteorder='big') + device_list
    return data


def read_time_message(upload_time, Vehicle, motor, locatoin, extremum, warining, enegy_devic_data, enegy_device_tmp):
    data = upload_time + Vehicle + motor + locatoin + extremum + warining + enegy_devic_data + enegy_device_tmp
    return data


def check_ack(cmd_id, data):
    # check start
    if not(data[0:].startswith(b'##')):
        print("Wrong start!")
        return 0
    # check id and flag
    if not(data[2] == cmd_id):
        print("Wrong ID!", data[2], "cmd:", cmd_id)
        return 0
    if not(data[3] == 1):
        print("ACK Fail!")
        return 0        
    return 1


# cmd_id 1 byte; ack_flag 1 byte; vin 17 bytes; encrypt_flag 1 byte; len 2 bytes.
def ev_car_msg(cmd_id, ack_flag, vin, encrypt_flag, length, data):
    start_bytes = b'\x23\x23'
    if length > 0:
        data = start_bytes+cmd_id+ack_flag+vin+encrypt_flag+length.to_bytes(2, byteorder='big')+data
    else:
        data = start_bytes+cmd_id+ack_flag+vin+encrypt_flag+length.to_bytes(2, byteorder='big')
    if ack_flag == b'\xFE':
        # command packet
        print("encode command")
    checksum = 0
    # for d in data[2:]:
    #     checksum = checksum ^ d
    it = iter(data)
    while True:
        try:
            checksum ^= next(it)
        except StopIteration:
            break
#    print(checksum)
    data += checksum.to_bytes(1, byteorder='big')
#    print(data)
    return data


# ev_car_data=ev_car_msg()
login_id = 1
login_data_tmp = login_data(current_time(), login_id, ICCID, 1, 1, b'1')
logout_data_tmp = logout_data(current_time(), login_id)
car_body_data = car_body(0x01, 0x03, 0x01, 0x0056, 0x0000010F, 0x01F1, 0x0100, 0x40, 0x01, 0x3E, 0x010E)
car_motor_data = driver_motor_data(0x01, driver_motor_assmbly_data(0x01, 0x01, 0x36, 0x00EF, 0x00E0, 0x2E, 0x1FE, 0x24))
car_location_data = location_data(0x05, 0x12312312, 0x01212123)
car_extremum_data = extremum_data(0x55, 0x33, 0x1212, 0x55, 0x11, 0x2323, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06)
car_warning_data = warning_data(0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x00, 0x00, 0x00, 0x00)
car_enegy_device_data = enegy_device_data(0x01, enegy_device_info(0x01, 0x12, 0x12, 0x12, 0x12, 0x08, b'\x00\x34\x00\x34\x00\x34\x00\x34\x00\x34\x00\x34\x00\x34\x00\x34'))
enegy_device_tmp_data = enegy_device_tmp(0x01, enegy_device_tmp_info(0x01, 0x08, b'\x11\x22\x11\x22\x11\x22\x11\x22\x11\x22\x11\x22\x11\x22\x11\x22'))
# Command id: login=1,realtime=2,backup=3,logout=4,platform_login=5,platform_logout=6
ev_car_heartbeat = ev_car_msg(b'\x07', b'\xFE', VIN, b'\x01', 0, 0)


ev_car_login = ev_car_msg(b'\x01', b'\xFE', VIN, b'\x01', len(login_data_tmp), login_data_tmp)
ev_car_logout = ev_car_msg(b'\x04', b'\xFE', VIN, b'\x01', len(logout_data_tmp), logout_data_tmp)


ev_car_vechicle_data_1 = vehicle_data(current_time(), b'\x01', car_body_data)
ev_car_realtime_data_1 = ev_car_msg(b'\x02', b'\xFE', VIN, b'\x01', len(ev_car_vechicle_data_1), ev_car_vechicle_data_1)
ev_car_backup_data_1 = ev_car_msg(b'\x03', b'\xFE', VIN, b'\x01', len(car_body_data), ev_car_vechicle_data_1)

realtime_data = read_time_message(current_time(), car_body_data ,car_motor_data , car_location_data , car_extremum_data , car_warning_data, car_enegy_device_data, enegy_device_tmp_data)
ev_realtime_data = ev_car_msg(b'\x02', b'\xFE', VIN, b'\x01', len(realtime_data), realtime_data)
# test_d = b"232301fe3335333831363035323738303237394e4501001e11030e0e1f3a010b383938363032423232323135303030333231393201009b"
# host = "218.205.176.44"
# port = 19006
host = "59.44.43.234"
port = 30032
# host = "127.0.0.1"
# port = 4001
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))
# s.sendto(ev_car_heartbeat,(host,port))
# s.settimeout(20.0)
# s.setblocking(True)
# print(s.recv(1))


# login ---------------------------------------------------------
s.sendto(ev_car_login, (host, port))
print("send data: ")
print(ev_car_login)
# print("hex print:")
# print(str(binascii.b2a_hex(ev_car_login))[2:-1] )
recv_data = (s.recv(1000))
print("receive data: ")
print(recv_data)
check_ack(1, recv_data)
# end login ----------------------------------------------------------
# while 1:
#    s.sendto(ev_car_heartbeat,(host,port))
#    recv_data=(s.recv(1000))
#    check_ack(7,recv_data)

# heartbeat data test ---------------------------------------------------
# s.sendto(ev_car_heartbeat, (host, port))
# print("send data: ")
# print(ev_car_heartbeat)
# print("hex print:")
# print(str(binascii.b2a_hex(ev_car_heartbeat))[2:-1] )
# recv_data = (s.recv(1000))
# print("receive data: ")
# print(recv_data)
# check_ack(7, recv_data)
# end heartbeat ----------------------------------------------------------

# realtime data ------------------------------------------------------
# realtime data test
s.sendto(ev_realtime_data, (host, port))
print("send data: ")
print(ev_realtime_data)
print("hex print:")
print(str(binascii.b2a_hex(ev_realtime_data))[2:-1] )
recv_data = (s.recv(1000))
print("receive data: ")
print(recv_data)
check_ack(2, recv_data)
# end readtime ----------------------------------------------------------

# backup data ----------------------------------------------------------
# backup data test
# s.sendto(ev_car_backup_data_1, (host, port))
# print("send data: ")
# print(ev_car_backup_data_1)
# # print("hex print:")
# # print(str(binascii.b2a_hex(ev_car_backup_data_1))[2:-1] )
# recv_data = (s.recv(1000))
# print("receive data: ")
# print(recv_data)
# check_ack(3, recv_data)
# end backup data------------------------------------------------------------

# logout -------------------------------------------------------------------
s.sendto(ev_car_logout, (host, port))
print(time.localtime(time.time()));
print("send data: ")
print(ev_car_logout)
recv_data = (s.recv(1000))
print(time.localtime(time.time()));
print("receive data: ")
print(recv_data)
check_ack(4, recv_data)
# end logout --------------------------------------------------------------

s.close()
