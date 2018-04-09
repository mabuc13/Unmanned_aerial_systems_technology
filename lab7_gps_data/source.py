from utm import utmconv
from math import acos, asin, sqrt, sin, cos, pi
from gps_functions import Gps_Functions

gps = Gps_Functions()

points = gps.get_read_from_file('position.log', 'lat', 'lon')
points = gps.get_remove_outliers(points)
gps.export_kml_data('outliers.kml', 'Outliers', points[:, 0], points[:, 1])

points = gps.get_read_from_file('position.log', 'lat', 'lon')
points = gps.get_remove_outliers(points)
points = gps.get_remove_by_angle(points, 15/180*pi)
print(len(points))
gps.export_kml_data('byAngle.kml', 'Angle', points[:, 0], points[:, 1])

points = gps.get_read_from_file('position.log', 'lat', 'lon')
points = gps.get_remove_outliers(points)
points = gps.get_number_points(points, 35)
gps.export_kml_data('byPoints.kml', 'Points', points[:, 0], points[:, 1])

# gps.export_to_QGC('mission.plan', points)