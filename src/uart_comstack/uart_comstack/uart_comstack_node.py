#!/usr/bin/env python3
"""
ComStack UART Interface for Raspberry Pi
Compatible with TM4C123 ComStack protocol

Protocol:
START(0xAA) | CMD | LEN | DATA | CHECKSUM | END(0x55)
"""

import serial
import struct
import time

# ===============================
# Protocol constants
# ===============================

START_BYTE = 0xAA
END_BYTE = 0x55

CMD_PING = 0x01
CMD_ACK = 0x02
CMD_NACK = 0x03
CMD_MOTOR_CMD = 0x10
CMD_MOTOR_STOP = 0x11
CMD_IMU_DATA = 0x22
CMD_ENCODER_DATA = 0x23

UART_PORT = "/dev/ttyAMA0"
BAUDRATE = 115200

MAX_DATA_LEN = 120

# ===============================
# UART Setup
# ===============================

ser = serial.Serial(
    port=UART_PORT,
    baudrate=BAUDRATE,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.1
)

# ===============================
# Checksum
# ===============================

def calculate_checksum(cmd, length, data):

    cs = cmd ^ length

    for b in data:
        cs ^= b

    return cs & 0xFF


# ===============================
# Packet Sender
# ===============================

def send_packet(command, data=b""):

    length = len(data)

    if length > MAX_DATA_LEN:
        raise ValueError("Data too large")

    checksum = calculate_checksum(command, length, data)

    packet = bytes([START_BYTE, command, length]) + data + bytes([checksum, END_BYTE])

    ser.write(packet)


# ===============================
# Motor Commands
# ===============================

def send_motor_command(left_speed, right_speed):

    if left_speed < -100 or left_speed > 100:
        raise ValueError("Left speed out of range")

    if right_speed < -100 or right_speed > 100:
        raise ValueError("Right speed out of range")

    data = struct.pack("<hh", left_speed, right_speed)

    send_packet(CMD_MOTOR_CMD, data)


def send_motor_stop():

    send_packet(CMD_MOTOR_STOP)


# ===============================
# Packet Receiver
# ===============================

def read_packet():

    while True:

        byte = ser.read(1)

        if len(byte) == 0:
            return None

        if byte[0] == START_BYTE:
            break

    header = ser.read(2)

    if len(header) < 2:
        return None

    cmd = header[0]
    length = header[1]

    if length > MAX_DATA_LEN:
        return None

    data = ser.read(length)

    if len(data) != length:
        return None

    footer = ser.read(2)

    if len(footer) < 2:
        return None

    checksum = footer[0]
    end_byte = footer[1]

    if end_byte != END_BYTE:
        return None

    return cmd, data


# ===============================
# Data Parsers
# ===============================

def parse_encoder(data):

    if len(data) != 16:
        return None

    left_ticks, right_ticks, left_vel, right_vel = struct.unpack("<iiff", data)

    return {
        "left_ticks": left_ticks,
        "right_ticks": right_ticks,
        "left_velocity": left_vel,
        "right_velocity": right_vel
    }


def parse_imu(data):

    if len(data) != 24:
        return None

    ax, ay, az, gx, gy, gz = struct.unpack("<6f", data)

    return {
        "accel": (ax, ay, az),
        "gyro": (gx, gy, gz)
    }


# ===============================
# Main Loop
# ===============================

def main():

    print("UART Communication Started")

    try:

        while True:

            packet = read_packet()

            if packet is None:
                continue

            cmd, data = packet

            if cmd == CMD_ENCODER_DATA:

                enc = parse_encoder(data)

                if enc:
                    print("ENCODER:", enc)

            elif cmd == CMD_IMU_DATA:

                imu = parse_imu(data)

                if imu:
                    print("IMU:", imu)

            elif cmd == CMD_PING:

                send_packet(CMD_ACK)

            else:

                print("Unknown command:", hex(cmd))

    except KeyboardInterrupt:

        print("Stopping...")

        send_motor_stop()

        ser.close()


# ===============================

if __name__ == "__main__":
    main()
