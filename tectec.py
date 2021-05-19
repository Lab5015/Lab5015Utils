from Lab5015_utils import read_arduino_temp
from Lab5015_utils import Keithley2450
from datetime import datetime
import time
import logging
from optparse import OptionParser

parser = OptionParser()
parser.add_option("-l","--log")
(options,args)=parser.parse_args()


logging.basicConfig(format='%(asctime)s %(message)s', datefmt='%Y-%m-%d %H:%M:%S',filename=options.log,level=logging.INFO)

timeout = 480
tensions = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 0.0, 0.0, 0.0]
mykey = Keithley2450()
mykey.set_V(0)
mykey.set_state(1)

init_time = datetime.now()

for tension in tensions:
    mykey.set_V(tension)

    print("The tension is: ", tension)
    
    while True:
        elapsed_time = datetime.now() - init_time
        temps = read_arduino_temp()

        (_, I, V) = mykey.meas_IV()

        out = str(I)+" "+str(V)
        for temp in temps:
            out += " "+temp
        
        logging.info(out)
        
        if elapsed_time.total_seconds() > timeout:
            init_time = datetime.now()
            break

mykey.set_state(0)
