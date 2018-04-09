#!/usr/bin/python
# -*- coding: utf-8 -*-

# IMU exercise
# Copyright (c) 2015-2018 Kjeld Jensen kjen@mmmi.sdu.dk kj@kjen.dk

##### Insert initialize code below ###################

## Uncomment the file to read ##
fileName = 'imu_razor_data_static.txt'
#fileName = 'imu_razor_data_pitch_55deg.txt'
#fileName = 'imu_razor_data_roll_65deg.txt'
#fileName = 'imu_razor_data_yaw_90deg.txt'

## IMU type
#imuType = 'vectornav_vn100'
imuType = 'sparkfun_razor'

## Variables for plotting ##
showPlot = True
plotDataP = []
plotDataR = []
plotDataFilter = []
plotDataGyro = []
timeArray = []

## Initialize your variables here ##
myValue = 0.0
myIntegrationVal = 0.0
sample_cnt = 0.0
delta_t = 0.0


######################################################

# import libraries
from math import pi, sqrt, atan2, atan
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, freqz
import numpy as np

def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


# open the imu data file
f = open(fileName, "r")

# initialize variables
count = 0

# looping through file

for line in f:
    count += 1

    # split the line into CSV formatted data
    line = line.replace('*', ',') #make the checkum another csv value
    csv = line.split(',')

    # keep track of the timestamps
    ts_recv = float(csv[0])
    if count == 1:
        ts_now = ts_recv# only the first time
        start_time = ts_now
    ts_prev = ts_now
    ts_now = ts_recv

    if imuType == 'sparkfun_razor':
        # import data from a SparkFun Razor IMU (SDU firmware)
        acc_x = int(csv[2]) / 1000.0 * 4 * 9.82;
        acc_y = int(csv[3]) / 1000.0 * 4 * 9.82;
        acc_z = int(csv[4]) / 1000.0 * 4 * 9.82;
        gyro_x = int(csv[5]) * 1/14.375 * pi/180.0;
        gyro_y = int(csv[6]) * 1/14.375 * pi/180.0;
        gyro_z = int(csv[7]) * 1/14.375 * pi/180.0;

    elif imuType == 'vectornav_vn100':
        # import data from a VectorNav VN-100 configured to output $VNQMR
        acc_x = float(csv[9])
        acc_y = float(csv[10])
        acc_z = float(csv[11])
        gyro_x = float(csv[12])
        gyro_y = float(csv[13])
        gyro_z = float(csv[14])

    #### Insert loop code below #########################

    # Variables available
    # ----------------------------------------------------
    # count		Current number of updates		
    # ts_prev	Time stamp at the previous update
    # ts_now	Time stamp at this update
    # acc_x		Acceleration measured along the x axis
    # acc_y		Acceleration measured along the y axis
    # acc_z		Acceleration measured along the z axis
    # gyro_x	Angular velocity measured about the x axis
    # gyro_y	Angular velocity measured about the y axis
    # gyro_z	Angular velocity measured about the z axis

    ## Insert your code here ##


    pitch = atan(acc_y/(sqrt(acc_x**2 + acc_z**2)))
    #roll = atan(-acc_x/acc_z)

    delta_t = ts_now - ts_prev
    gyro_x-= 0.00074155
    myIntegrationVal += gyro_x * delta_t
    timeArray.append(sample_cnt)
    sample_cnt += 1

    #print(ts_now)


    myValueP = pitch # relevant for the first exercise, then change this.
    #myValueR = roll # relevant for the first exercise, then change this.

    # in order to show a plot use this function to append your value to a list:
    plotDataGyro.append(myIntegrationVal)#*180.0/pi)
    plotDataP.append(myValueP*180.0/pi)
    #plotDataR.append(myValueR*180.0/pi)

    ######################################################

# closing the file
f.close()

#lowpass filter
data = plotDataP
cutoff = 0.2
fs= 100
order=3
lowpass_p = butter_lowpass_filter(plotDataP, cutoff, fs, order=5)
lowpass_r = butter_lowpass_filter(plotDataR, cutoff, fs, order=5)

data = np.polyfit(timeArray, plotDataGyro, 1)
print('data: ')
print(data)

variance = np.var(plotDataP)
print(variance)

# show the plot
#if showPlot == True:
#    FilterPlot = plt.plot(timeArray, plotDataGyro, label = 'Gyro pos')
#    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
#        ncol=2, mode="expand", borderaxespad=0.)
#    plt.savefig('imu_exercise_plot_GyroPos_ex4-31.png')
#    plt.show()
