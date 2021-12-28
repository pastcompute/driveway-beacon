import serial
import re
import time
import math

ser = serial.Serial('/dev/ttyUSB1', 115200, parity=serial.PARITY_NONE, stopbits=1, rtscts=0, xonxoff=0, timeout=None)
ser.flushInput()

pat = re.compile(" ")

rssi_valid = False
t0 = 0
t1 = 0
nn = 0

while True:
    nn = nn + 1
    try:
        bytes = ser.readline()
        t1 = time.time()
        if len(bytes) < 3:
            # obviously not hex address:
            #print(bytes)
            rssi_valid = False
            continue
        s = bytes.decode("utf-8").rstrip().lstrip()
        #print(nn, s)
        if s[0:3] != '00:':
            if s[0:3] == "Rx ":
                # Note: rssi_valid should now be true...
                continue
            if s[0:6] == "[DBUG]":
                m = re.search(r"rssi_pkt=(-{0,1}[0-9][0-9]*) snr_pkt=(-{0,1}[0-9][0-9]*)", s)
                if (m):
                    #print(m.group(1), m.group(2))
                    #print("rssi,%s,%s" % (m.group(1), m.group(2)))
                    rssi = int(m.group(1))
                    snr = int(m.group(2))
                    t0 = t1
                    rssi_valid = True
                continue
            # either an error or debug we can filter out, or work out how to handle larger packets later
            print("UNKNOWN,%s" % bytes.decode("utf-8").rstrip().lstrip())
            rssi_valid = False
            continue
        bytes2 = pat.split(s[4:])
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

        if t0 > 0:
            t1 = t0  # use earlier time if we parsed an rssi msg before the daat

        g = time.gmtime(t1)
        t = time.strftime("%Y-%m-%dT%H:%M:%S.", g)
        ms = math.modf(t1)[0] * 1000
        if not rssi_valid:
            rssi = -1
            snr = -1
        print("data,%.03f,%s%03d,%d,%d,%d,%d,%d,%s,%s" % (t1, t, ms, counter, ticks10*10, magnitude, tempC, baseline, rssi, snr))
        rssi_valid = False
        t0 = 0
    except KeyboardInterrupt as e:
        break
    except Exception as e2:
        print(e2)
        break
