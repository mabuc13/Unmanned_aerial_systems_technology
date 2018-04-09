# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
from math import acos, asin, sqrt, sin, cos, pi, atan

speed = 3*10**8 #Speed of light in meter /s

x_values = []

p1 = []
lambda1 = speed/(2.4*10**9)
print "Lambda1: ", lambda1

p2 = []
freq = 433*(10**6)
lambda2 = 0.69284064665 #Couldnt get python to get the correct value for this one, so i used a calculator...
print("Lambda2: ", lambda2)
print (3*(10**8))/(433*(10**6))

p3 = []
lambda3 = speed/(5.8*10**9)
print("Lambda3: ", lambda3)

for x in xrange(1, 2000):
    x = x/10
    Fn1 = sqrt( ( lambda1*x*(200-x))/(x+200-x))
    Fn2 = sqrt((lambda2*x*(200-x))/(x+200-x))
    Fn3 = sqrt((lambda3*x*(200-x))/(x+200-x))
    p1.append(Fn1)
    p1.append(-Fn1)
    p2.append(Fn2)
    p2.append(-Fn2)
    p3.append(Fn3)
    p3.append(-Fn3)
    x_values.append(x)
    x_values.append(x)

plt.plot(x_values, p1, label = '2.4 GHz', linestyle="",marker=".")
plt.plot(x_values, p2, label = '433 MHz',linestyle="",marker=".")
plt.plot(x_values, p3, label = '5.8 GHz',linestyle="",marker=".")

plt.xlabel('[Meters]')
plt.ylabel('[Meters]')

plt.title("First Fresnel zone for different frequencies")

plt.legend()

plt.show()