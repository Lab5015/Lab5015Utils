#! /usr/bin/python3

from Lab5015_utils import PiLas
from optparse import OptionParser

parser = OptionParser()

parser.add_option("--power", dest="power", default="0")
parser.add_option("--freq", dest="freq", default="10000")
parser.add_option("--tune", dest="tune", default="0")
(options, args) = parser.parse_args()



las = PiLas()

print("set tune:")
las.set_tune(str(float(options.tune)*10.))
print("set freq:")
las.set_freq(options.freq)
print("set power:")
las.set_state(options.power)

