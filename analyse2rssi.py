# Take the input and graph rssi over time

import sys
import numpy as np
import pandas as pd
from subprocess import Popen,PIPE
import csv
import plotext as plx

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

sample = 0
prior_uptime = 0
prior_epoch = 0

Asnr = np.empty(86400 * 2 * 10)
Arssi = np.empty(86400 * 2 * 10)
Atemp = np.empty(86400 * 2 * 10)
At = np.empty(86400 * 2 * 10)

def parse_line(line):

    for row in csv.reader([line]):
        values = row
    epoch = float(values[1])
    uptime = int(values[4])
    tempC = int(values[6])
    rssi = int(values[8])
    snr = int(values[9])

    global sample, prior_uptime, prior_epoch, Arssi, Atemp, At, Asnr, t0

    if sample == 0:
        t0 = epoch

    Arssi[sample] = rssi
    Atemp[sample] = tempC
    Asnr[sample] = snr
    At[sample] = epoch - t0

    if sample > 0:
        if prior_uptime > uptime:
            print("Reboot detected at %d samples (time diff=%d)" % (sample, (epoch - prior_epoch)))
    prior_uptime = uptime
    prior_epoch = epoch

    sample = sample + 1


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

Arssi = Arssi[0:sample]
Atemp = Atemp[0:sample]
At = At[0:sample]

print("samples=%d" % (sample))
print("duration=%d" % (prior_epoch - t0))
print("rssi min=%d max=%d" % (Arssi.min(), Arssi.max()))
print("snr min=%d max=%d" % (Asnr.min(), Asnr.max()))
print("Temp min=%d max=%d" % (Atemp.min(), Atemp.max()))

#print(At)
#print(Arssi)

plx.scatter(At, Arssi)
plx.scatter(At, Atemp)
plx.show()
