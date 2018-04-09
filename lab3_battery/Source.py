import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import numpy.polynomial.polynomial as poly
import math

# Plot capacity and weight of the batteries.
if True:
    xl = pd.ExcelFile("MELASTA_Li-Polymer.xlsx")
    df = xl.parse("Sheet1")

    df1 = df[df.columns[3:5]]
    df1 = df1[df1.index != 0]
    df1 = df1[df1.index != 1]
    df1 = df1[df1.index != 30]
    df1 = df1[df1.index != 35]
    df1 = df1[df1.index != 103]
    df1 = df1[df1.index != 130]
    df1 = df1[df1.index != 131]
    df1 = df1[df1.index != 132]

    df1['Unnamed: 4'] = df1['Unnamed: 4'].map(lambda x: x.split('±', 1)[0])

    df1 = df1.astype(float)

    print(df1)

    corr = df1.corr()

    print(corr)

    #plt.plot(df1['Unnamed: 3'], df1['Unnamed: 4'], 'ro')
    #plt.show()

    fig = plt.figure()
    plt.plot(df1['Unnamed: 3'], df1['Unnamed: 4'], 'ro')
    fig.suptitle('Capacity and weight of Li-Po batteries', fontsize=20)
    plt.xlabel('Capacity (mAh)', fontsize=18)
    plt.ylabel('Weight (g)', fontsize=16)
    #fig.savefig('test.jpg')
    plt.show()

# Plot voltage as a function of time.
if False:
    data = pd.read_csv('log-2016-01-14.txt', sep="	", header=None)
    df = data[data.columns[[4,11]]]

    df.columns = ["Time", "Voltage"]

    fig = plt.figure()
    plt.plot(df['Time'], df['Voltage'], 'ro')
    fig.suptitle('Discharge curve of battery', fontsize=20)
    plt.xlabel('GNSS time', fontsize=18)
    plt.ylabel('Voltage', fontsize=16)
    plt.show()

# Transform the data and make fits to estimate SOC.
if True:
    data = pd.read_csv('log-2016-01-14.txt', sep="	", header=None)
    df = data[data.columns[[4,11]]]
    print(df)

    df.columns = ["Time", "Voltage"]

    toZero = 114854
    gap2init = 115959 - toZero
    gap2 = 120002 - 115959
    gap3init = 125956 - toZero - gap2
    gap3 = 130004 - 125956
    gap4init = 135958 - toZero - gap3 - gap2
    gap4 = 140001 - 135958
    df["Time"] = df["Time"] - 114854
    df["Time"] = df["Time"].apply(lambda x: x - gap2 if x > gap2init else x)
    df["Time"] = df["Time"].apply(lambda x: x - gap3 if x > gap3init else x)
    df["Time"] = df["Time"].apply(lambda x: x - gap4 if x > gap4init else x)

    totalDif = 144514 - 114854 - gap2 - gap3 - gap4

    df["Time"] = df["Time"] / totalDif * 100

    divider1 = 3400
    divider2 = 600

    volFirstLast = np.array([df.ix[4215, 'Voltage'], df.ix[0, 'Voltage']])
    timeFirstLast = np.array([df.ix[0, 'Time'], df.ix[4215, 'Time']])

    vol1 = np.array([df.ix[4215, 'Voltage'], df.ix[divider1, 'Voltage']])
    time1 = np.array([df.ix[0, 'Time'], df.ix[4215-divider1, 'Time']])

    vol2 = np.array([df.ix[divider1, 'Voltage'], df.ix[divider2, 'Voltage']])
    time2 = np.array([df.ix[4215-divider1, 'Time'], df.ix[4215-divider2, 'Time']])

    vol3 = np.array([df.ix[divider2, 'Voltage'], df.ix[0, 'Voltage']])
    time3 = np.array([df.ix[4215-divider2, 'Time'], df.ix[4215, 'Time']])

    #print(poly.polyfit(df['Voltage'], df['Time'][::-1], 4))
    firstLastVals = poly.polyfit(volFirstLast, timeFirstLast, 1)
    part1vals = poly.polyfit(vol1, time1, 1)
    part2vals = poly.polyfit(vol2, time2, 1)
    part3vals = poly.polyfit(vol3, time3, 1)

    t = np.linspace(3.19, 4.15, 100)
    t1 = np.linspace(vol1[0], vol1[1], 100)
    t2 = np.linspace(vol2[0], vol2[1], 100)
    t3 = np.linspace(vol3[0], vol3[1], 100)

    lineFirstLast = firstLastVals[0] + firstLastVals[1] * t
    splitPart1 = part1vals[0] + part1vals[1]*t1
    splitPart2 = part2vals[0] + part2vals[1]*t2
    splitPart3 = part3vals[0] + part3vals[1]*t3

    fig = plt.figure()
    plt.plot(df['Voltage'], df['Time'][::-1], 'ro')
    plt.plot(t, lineFirstLast, 'lightgreen')
    plt.plot(t1, splitPart1, 'b')
    plt.plot(t2, splitPart2, 'b')
    plt.plot(t3, splitPart3, 'b')
    fig.suptitle('Estimating SOC with voltage', fontsize=20)
    plt.xlabel('Voltage', fontsize=18)
    plt.ylabel('SOC', fontsize=16)
    plt.legend(['Data points', 'Linear fit from start- to endpoint', 'Divided into three linear fits'], loc='upper left')
    plt.show()

    print(firstLastVals)
    print(part1vals)
    print(part2vals)
    print(part3vals)

    print(vol1)
    print(vol2)
    print(vol3)