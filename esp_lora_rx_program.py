import board
import digitalio
import time
import serial

reset = digitalio.DigitalInOut(board.D2)
reset.direction = digitalio.Direction.OUTPUT

program = digitalio.DigitalInOut(board.D22)
program.direction = digitalio.Direction.OUTPUT

reset.value = False
program.value = False

print("Reset ESP8266 with origram:=0")
time.sleep(0.1)
reset.value = True
print("Release reset")
time.sleep(3)
program.value = True
print("Release program")
