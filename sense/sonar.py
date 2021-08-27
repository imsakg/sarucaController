import brping
import time

myPing = brping.Ping1D()
myPing.connect_serial("/dev/ttyUSB0", 115200)

if myPing.initialize() is False:
    print("Failed to initialize Ping!")
    exit(1)

while True:
    data = myPing.get_distance()
    myPing.set_speed_of_sound(340000)
    if data:
        print(data)
    else:
        print("Failed to get distance data")
