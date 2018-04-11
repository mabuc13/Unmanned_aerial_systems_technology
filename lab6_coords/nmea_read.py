#!/usr/bin/python
#/****************************************************************************
# nmea read function
# Copyright (c) 2018, Kjeld Jensen <kj@kjen.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
'''
2018-03-13 Kjeld First version, that reads in CSV data
'''

import exportkml
import matplotlib.pyplot as plt
import pylab
from bokeh.plotting import figure, show, output_file


class nmea_class:
    def __init__(self):
        self.data = []

    def import_file(self, file_name):
        file_ok = True
        try:
            # read all lines from the file and strip \n
            lines = [line.rstrip() for line in open(file_name)]
        except:
            file_ok = False
        if file_ok == True:
            pt_num = 0
            for i in range(len(lines)): # for all lines
                if len(lines[i]) > 0 and lines[i][0] != '#': # if not a comment or empty line
                    csv = lines[i].split (',') # split into comma separated list
                    self.data.append(csv)

    def print_data(self):
        for i in range(len(self.data)):
            print(self.data[i])


if __name__ == "__main__":
    #print('Importing file')
    # importing files into main
    nmea = nmea_class()
    nmea.import_file ('nmea_trimble_gnss_eduquad_flight.txt')
    static = nmea_class()
    static.import_file('nmea_ublox_neo_24h_static.txt')

    # create files to be shown on maps
        #exportkml class used
    kml = exportkml.kmlclass()
    kml.begin('drone_flight.kml', 'drone_flight', 'the flightplan of a drone', 0.1)
    sta = exportkml.kmlclass()
    sta.begin('static.kml', 'static_err', 'this is the error in the static postion of the dron within 24 hours', 0.1)

    # color: use 'red' or 'green' or 'blue' or 'cyan' or 'yellow' or 'grey'
    # altitude: use 'absolute' or 'relativeToGround'
    kml.trksegbegin ('', '', 'red', 'absolute')
    #kml.trkpt(latitude, Longitude, elevation) #Too see what the components are
    time = []
    height = []
    satellite_cnt = []
    
    print('Creating the file drone_flight.kml as an example on how tu use kmlclass')
    
    for i in range(len(nmea.data)):
        if(static.data[i][0] == '$GPGGA'):
            #x = nmea.data[i].readline()
            #kml.trkpt(x[4], x[2], x[8])
            lat = float(nmea.data[i][2])/100
            lon = float(nmea.data[i][4])/100
            ele = float(nmea.data[i][11])
            kml.trkpt(lat, lon, ele)
            time.append(float(nmea.data[i][1]))
            height.append(float(nmea.data[i][9]))
            satellite_cnt.append(float(nmea.data[i][7]))
            #print("The drone is " + str(nmea.data[i][9]) + " meter above sea level at time: " + nmea.data[i][1])
            #print("and " + str(nmea.data[i][7]) + " satelites are used to track the droe at time: " + nmea.data[i][1])
    #pylab.plot(time,height)
    #pylab.ylabel('height')
    #pylab.xlabel('time')
    #pylab.show()
    
    #print(height)
    x = [1, 2, 3, 4, 5]
    y = [6, 7, 2, 4, 5]
    output_file('height.html')
    p = figure(title="height vs time", x_axis_label='time', y_axis_label='height')
    p.line(time, height, legend="height.", line_width=2)
    show(p)
    
    output_file('satellites.html')
    t = figure(title="satellites vs time", x_axis_label='time', y_axis_label='height')
    t.line(time, satellite_cnt, legend="satellites.", line_width=2)
    show(t)
    
    #print('Creating the file static.kml that describes the error in GNSS accuracy')
    # sta.trksegbegin ('', '', 'blue', 'absolute')
    # for i in range(len(static.data)):
        # if(static.data[i][0] == '$GPGGA'):
            # if(static.data[i][2] == ''):
                # continue
            # lat = float(static.data[i][2])/100
            # lon = float(static.data[i][4])/100
            # ele = float(static.data[i][11])
            # sta.trkpt(lat, lon, ele)

    sta.trksegend()
    sta.end()
    kml.trksegend()
    kml.end()

