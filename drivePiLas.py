#!/usr/bin/env python3

from pilas import PiLas
from optparse import OptionParser

parser = OptionParser()

parser.add_option("--power",   dest="power", default="0")
parser.add_option("--freq",    dest="freq", default="50000")
parser.add_option("--tune",    dest="tune", default="50")
parser.add_option("--trigger", dest="trigger", default="1")
(options, args) = parser.parse_args()


las = PiLas()

print("set tune to " + str(options.tune) + "%")
las.set_tune(str(float(options.tune)*10.))
print("set freq to " + options.freq + " Hz")
las.set_freq(options.freq)
print("set trigger to " + options.trigger)
las.set_trigger(options.trigger)
print("power is " + options.power)
las.set_state(options.power)
