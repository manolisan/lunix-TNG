import os
import sys
import numpy as np
from matplotlib import pyplot as plt
fd = os.open(str(sys.argv[1]), os.O_RDONLY)

plt.ion() # set plot to animated

ydata = [0] * 50
#ax1=plt.axes()

# make plot
line, = plt.plot(ydata)
plt.ylim([0,40000])

full_data = ""
# start data collection
while True:
    data = os.read(fd,6).split("\n")[0] # read data from port
    print data
    full_data = full_data + data
    if len(full_data) >= 5:
        printable = (full_data[:2] + '.' + full_data[2:5])
        number = float(printable)
                                       # port and strip line endings
        print "Full data: " + full_data
        #if len(data.split(".")) == 2:
        ymin = float(min(ydata))-10
        ymax = float(max(ydata))+10
        plt.ylim([ymin,ymax])
        ydata.append(number)
        del ydata[0]
        line.set_xdata(np.arange(len(ydata)))
        line.set_ydata(ydata)  # update the data
        plt.draw() # update the plot

        full_data = ""
        # to FIX
        #if len(full_data) == 5:
        #   full_data = ""
        #else:
        #    full_data = full_data[5:]
