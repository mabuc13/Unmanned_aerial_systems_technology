# -*- coding: utf-8 -*-
from utm import utmconv
from math import acos, asin, sqrt, sin, cos, pi, atan
import matplotlib.pyplot as plt
import numpy.polynomial.polynomial as poly
import numpy as np

f = open ('position.log', "r")

#Variables for plotting
showPlot = True
latData = []
lonData = []
timeData = []
utm_easting = []
utm_northing = []
distance = []
variable_distance = 0
# initialize variables
count = 0
uc = utmconv()
oldE = 590686.5315567022
oldN = 6136529.126779066
# looping through file
for line in f:
    count += 1
    # split the line into CSV formatted data
    line = line.replace ('*',',') # make the checksum another csv value
    csv = line.split(',')
    time = float(csv[0])
    lat = float(csv[1])
    lon = float(csv[2])
    #latData.append(lat)
    #lonData.append(lon)
    #timeData.append(time)
    (hemisphere, zone, letter, easting, northing) = uc.geodetic_to_utm(lat, lon)
    #utm_easting.append(easting)
    #utm_northing.append(northing)
    tmpDistance = sqrt((oldE-easting)**2+(oldN-northing)**2)

    if tmpDistance < 1+variable_distance and time < 527.542:
        utm_easting.append(easting)
        utm_northing.append(northing)
        timeData.append(time)
        distance.append(tmpDistance)
        variable_distance = 0
        oldE = easting
        oldN = northing
    else:
        variable_distance += 0.2
f.close()

lessPointsE = []
lessPointsN = []
count = 0
for i in  utm_easting:
    if count == 0:
        referenceE = i
        referenceN = utm_northing[count]
        referenceAngle = atan(referenceN-utm_northing[1]/ referenceE-utm_easting[1])
    else:
        currentE = i
        currentN = utm_northing[count]
        currentAngle = atan(referenceN-currentN/referenceE -currentE)
    if count > 1:
        print(currentAngle-referenceAngle)
        if currentAngle-referenceAngle > 5*180/pi:
            print(currentE)
            lessPointsE.append(currentE)
            lessPointsN.append(currentN)
            referenceE = currentE
            referenceN = currentN
            referenceAngle = currentAngle
    count += 1

plt.plot(lessPointsE, lessPointsN, label = "Lat vs lon", linestyle="",marker=".")
plt.show()