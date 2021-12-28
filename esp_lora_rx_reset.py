import board
import digitalio
import time
import serial

reset = digitalio.DigitalInOut(board.D2)
reset.direction = digitalio.Direction.OUTPUT
program = digitalio.DigitalInOut(board.D22)
program.direction = digitalio.Direction.OUTPUT
program.value = True

reset.value = False
time.sleep(0.1)
reset.value = True

print("Waiting for serial")
time.sleep(3)

print("Adjusting bw +1 ~> 125000 and sf 9 ~> 7")
ser = serial.Serial('/dev/ttyAMA0', 115200, parity=serial.PARITY_NONE, stopbits=1, rtscts=0, xonxoff=0, timeout=None)

ser.write(b'b')
time.sleep(0.5)

for n in range(0,5):
    ser.write(b's')
    time.sleep(0.5)

ser.write(b'h')
print("ok")

# Odd - for some reason it still wont start the flashing LED indicating reception
# until we start picocom and press h
