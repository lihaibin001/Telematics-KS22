import sys
import os
import binascii


def char2hex():
    return binascii.hexlify(b'seed')


while 1:
    # get seed
    seed = input("seed: ")
    # encode the HEX data
    hex_seed = '0x' + seed
    # convert HEX to INT
    int_seed = int(hex_seed, 16)
    # calculate key
    oct_key = ((((int_seed >> 4) ^ int_seed) << 3) ^ int_seed) & 0xFFFFFFFF
    # convert INT to HEX
    hex_key = hex(oct_key)
    # print HEX key
    print(hex_key)
    for index in range(8):
        print(hex_key[index + 2], end="")
        if index % 2 == 1:
            print(' ', end="")
    print("")

