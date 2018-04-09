import numpy as np
import pandas as pd
from bokeh.models import CustomJS, ColumnDataSource
from bokeh.models.widgets import Button
from bokeh.plotting import figure, output_file, show
import numpy.polynomial.polynomial as poly
import scipy.stats

df = pd.read_csv("CSV.txt")

plot = figure(plot_width=400, plot_height=400)
plot = figure(title="Data from the tests")
plot.grid.grid_line_alpha=0.3
plot.xaxis.axis_label = 'Duty cycle'
plot.yaxis.axis_label = 'Grams'

average = (df['CW_Small1'] + df['CW_Small2'] + df['CW_Small3']) / 3

plot.line(df['Duty_cycle'], df['CW_Small1'], color='red', legend='CW_Small_1')
plot.line(df['Duty_cycle'], df['CW_Small2'], color='green', legend='CW_Small_2')
plot.line(df['Duty_cycle'], df['CW_Small3'], color='blue', legend='CW_Small_3')
plot.line(df['Duty_cycle'], df['CW_Big'], color='black', legend='CW_Big')
plot.line(df['Duty_cycle'], df['CCW_Big'], color='orange', legend='CCW_Big')
plot.legend.location = "top_left"

#show(plot)

regression = poly.polyfit(df['Duty_cycle'], average, 1)


print(regression)

t = np.linspace(0, 100, 100)

regressionLine = regression[0] + regression[1] * t

plot_reg = figure(plot_width=400, plot_height=400)
plot_reg = figure(title="Average of the small rotor and the result of regression")
plot_reg.grid.grid_line_alpha=0.3
plot_reg.xaxis.axis_label = 'Duty cycle'
plot_reg.yaxis.axis_label = 'Grams'

plot_reg.line(df['Duty_cycle'], average, color='purple', legend='Average')
plot_reg.line(t, regressionLine, color='orange', legend='Linear function, y = 8.29x - 51.20')
plot_reg.legend.location = "top_left"

#show(plot_reg)

df_bat = pd.read_csv("Battery.txt")
plot_bat = figure(plot_width=400, plot_height=400)
plot_bat = figure(title="Voltage drops of the battery")
plot_bat.grid.grid_line_alpha=0.3
plot_bat.xaxis.axis_label = 'Test number'
plot_bat.yaxis.axis_label = 'Voltage'

plot_bat.circle(df_bat['Test'], df_bat['Before'], color='blue', legend='Voltage before testing', radius=0.05)
plot_bat.circle(df_bat['Test'], df_bat['After'], color='red', legend='Voltage after testing', radius=0.05)
plot_bat.legend.location = "top_center"

show(plot_bat)

slope, intercept, r_value, p_value, std_err = scipy.stats.linregress(df['Duty_cycle'], average)
print(r_value**2)

