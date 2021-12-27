import serial
import re
ser = serial.Serial('/dev/ttyUSB1', 115200, parity=serial.PARITY_NONE, stopbits=1, rtscts=0, xonxoff=0, timeout=None)
ser.flushInput()

pat = re.compile(" ")

while True:
    try:
        bytes = ser.readline()
        #decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
        #print(decoded_bytes)
        if len(bytes) < 3:
            # obviously not hex address:
            continue
        aa = bytes[0:3]
        if aa != "00:":
            # work out how to handle larger packets later
            continue
        bytes2 = pat.split(bytes[4:])
        payload_len = int(bytes2[0], 16)
        if len(bytes2) < payload_len or payload_len < 4:
            print("Payload rx length error: " + bytes2)
            continue
        if (bytes2[1] != '5f'):
            print("Payload magic error: " + bytes2)
            continue
        if (bytes2[2] != '01'):
            print("Payload type error: " + bytes2)
            continue
        if payload_len < 12:
            print("Payload type length error: " + bytes2)
            continue
        counter = int(bytes2[4], 16) + int(bytes2[3], 16) * 256
        ticks10 = int(bytes2[5], 16) + int(bytes2[6], 16) * 256 + int(bytes2[7], 16) * 65536
        magnitude = int(bytes2[8], 16) + int(bytes2[9], 16) * 256
        tempC = int(bytes2[10], 16)
        baseline = int(bytes2[11], 16)

        print counter, ticks10*10, magnitude, tempC, baseline



    except KeyboardInterrupt as e:
        break
    except Exception as e2:
        print(e2)
        break
