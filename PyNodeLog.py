import serial
import time
import csv

# For OSX w/platformio
# RocketScream
# usb_port = "/dev/cu.usbmodem142101"
# ESP8266
usb_port = "/dev/tty.SLAB_USBtoUART"

ser = serial.Serial(usb_port, 115200)
ser.flushInput()

while True:
    try:
        ser_bytes = ser.readline()
        # decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
        decoded_bytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
        print(decoded_bytes)
        with open("test_data.csv", "a") as f:
            writer = csv.writer(f, delimiter=",")
            writer.writerow([time.time(), decoded_bytes])
    except:
        print("Keyboard Interrupt")
        break
