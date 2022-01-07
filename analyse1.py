# Take the input from (etc) and try and resolve down to:
# - two car entry/exits
# strings mlog5.txt|egrep ^data | egrep -v ',7[0-9],[2-4][0-9],79' |grep -v ,8[0-3],[2-4]
# - one delivery truck entry/exit
# strings mlog6.txt|egrep ^data | egrep -v ',7[0-9],[2-4][0-9],79' |grep -v ,8[0-3],[2-4]

import sys
import numpy as np
import pandas as pd
from subprocess import Popen,PIPE
import csv
    
if len(sys.argv) < 2:
    print("Please specify filename")
    sys.exit(0)
datafile = sys.argv[1]

# Run through strings and egrep for ^data, for now
# THen we can chop it for testing as well

# this is equivalent to ls -lha | grep "foo bar"
p1 = Popen(["strings", datafile], stdout=PIPE)
p2 = Popen(["egrep", "^data,"], stdin=p1.stdout, stdout=PIPE)
p1.stdout.close()
#p3 = Popen(["head", "-2000000"], stdin=p2.stdout, stdout=PIPE)
#p2.stdout.close()

long_sum = 0
short_sum = 0
mag_min = 9999999
mag_max = 0
tempC_min = 999
tempC_max = -999

# Number of samples to integrate to generate the detection threshold
shortrun_sample_count = 18
detection_reset_delay_count = 5 # approx 90 samples or 9 seconds?

long_count = 0
short_count = 0
prior_short_average = -1
short_average = 0
rebootDetected = False
prior_uptime = 0
st0 = 0
detection = 0
successive_det_count = 0
successive_nodet_count = 0
prior_epoch = 0
latchPeak = 0
latchMin = 999999
latched = False
nlatched = 0

# try and integrate so we can have a guessuristic for a car vs truck, etc.
# although it wont take into account the vehicle speed, so... (unless we do integral sum per unit time, which is getting a bit hard at the moment)
# I mean the same vehicle generated twice as much "energy" going the other direction
# So the reality is more complex
integral = 0

def parse_line(line):

    for row in csv.reader([line]):
        values = row
    #print(values[1],values[5],values[6])
    epoch = float(values[1])
    stamp = values[2]
    count = int(values[3])
    uptime = int(values[4])
    mag = int(values[5])
    tempC = int(values[6])

    global long_sum, long_count, short_sum, short_count, prior_uptime, short_average, st0, prior_short_average
    global detection, successive_det_count, successive_nodet_count, latched, integral, latchPeak, latchMin, nlatched, prior_epoch

    if long_count == 0:
        st0 = epoch

    # This is a rough average, because it doesnt take into account time gaps...
    long_sum = long_sum + mag
    long_count = long_count + 1

    # Perhaps we should drop data where the time is not contiguous?
    short_sum = short_sum + mag
    short_count = short_count + 1

    nlatched = nlatched + 1

    if long_count > 0:
        if prior_uptime > uptime:
            print("Reboot detected at %d samples" % long_count)
    prior_uptime = uptime

    if prior_short_average > 0:
        if detection == 0:
            integral = 0

        # Sum the entire range from first possible movement to 5 zones later
        integral = integral + abs(mag - prior_short_average)
        if (mag > prior_short_average + 4) or (mag < prior_short_average - 4):
            # We need to invalide the current one somehow...
            detection = 1
            detected = False
            successive_det_count = successive_det_count + 1
            # Note - for a true detection - need to have 2 in a group, not just a single point
            # This does mean the printout ignores the first sample...
            # Which doesnt matter for pure detection
            # But matters if we are gathering training data
            # OR a 101
            if successive_det_count >= 2:
                detected = True
            elif successive_nodet_count == 1:
                detected = True
            successive_nodet_count = 0
            # How to tell ...10101....
            if detected:
                print("Detection @ %.2f, %s, short_av %.2f, prior_short_av %.2f, mag %d " % (epoch - st0, stamp, short_average, prior_short_average, mag))
                if not latched:
                    print("DETECTION LATCHED AT %s" % (stamp))
                latched = True
                nlatched = 1
            else:
                print("FalseDet  @ %.2f, %s, short_av %.2f, prior_short_av %.2f, mag %d " % (epoch - st0, stamp, short_average, prior_short_average, mag))
        else:
            successive_det_count = 0
            if detection == 1:
                successive_nodet_count = successive_nodet_count + 1
                # So a single 0 in the middle will get counted
 
        if detection == 1:
            if latchPeak < mag:
                latchPeak = mag
            if latchMin > mag:
                latchMin = mag


    if short_count == shortrun_sample_count:
        # background doesnt get updated whilst a detection is bouncing around
        if detection == 0: 
            prior_short_average = short_average
        short_average = short_sum / short_count 
        short_sum = 0
        short_count = 0
        detection = detection + 1
        # Give it some time after a detection to stabilise again
        # Although this wont help with a car parked on top...
        if detection == detection_reset_delay_count:
            detection = 0
            successive_det_count = 0
            successive_nodet_count = 0
            if latched:
                print("DETECTION CONCLUDED, INTEGRAL=%d PEAK=%d MIN=%d nlatched=%d" %(integral, latchPeak, latchMin, nlatched))
            integral = 0
            latchPeak = 0
            latchMin = 999999
            latched = False

    prior_epoch = epoch

    if (long_count % 60000) == 0:
        print("Time %d Samples %d Long Average %d Last Short Average %.2f" % (epoch - st0, long_count, long_sum / long_count, short_average))
        pass

cont = True
while cont:
    cont = False
    line = p2.stdout.readline()
    if not line == b"":
        out = line.decode("utf-8").rstrip()
        parse_line(out)
        cont = True

    if not cont and p2.poll() is not None:
        break

print("Time %d Samples %d Long Average %d Last Short Average %.2f" % (prior_epoch - st0, long_count, long_sum / long_count, short_average))

#df=pd.read_csv(datafile, sep=',', header=None)
#df.ftypes
#len(df.values))
