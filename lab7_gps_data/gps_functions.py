from utm import utmconv
from math import acos, asin, sqrt, sin, cos, pi, atan, atan2
import numpy as np
import pandas as pd
from bokeh.plotting import figure, output_file, show
from exportkml import kmlclass
from numpy.linalg import norm
import json

class Gps_Functions:

    def __init__(self):
        self.df = 0

    def distance_between_points(self, lat1, lon1, lat2, lon2):
        lat1 = lat1 * pi / 180
        lat2 = lat2 * pi / 180
        lon1 = lon1 * pi / 180
        lon2 = lon2 * pi / 180

        distance_radians = acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(lon1 - lon2))
        distance_km = distance_radians * 6371.0
        return distance_km

    def get_read_from_file(self, filepath, lat_name, lon_name):
        df = pd.read_csv(filepath)
        lat_data = df[lat_name]
        lon_data = df[lon_name]
        points = np.column_stack((lat_data, lon_data))
        return points

    def plot(self, x_name, y_name, title, x_label, y_label, legend):
        plot = figure(plot_width=600, plot_height=600, title=title)
        plot.grid.grid_line_alpha = 0.3
        plot.xaxis.axis_label = x_label
        plot.yaxis.axis_label = y_label

        plot.circle(self.df[x_name], self.df[y_name], color='red', legend=legend)
        plot.legend.location = "top_left"

        show(plot)

    def plot_data(self, x_data, y_data, title, x_label, y_label, legend):
        plot = figure(plot_width=600, plot_height=600, title=title)
        plot.grid.grid_line_alpha = 0.3
        plot.xaxis.axis_label = x_label
        plot.yaxis.axis_label = y_label

        plot.circle(x_data, y_data, color='red', legend=legend)
        plot.legend.location = "top_left"

        show(plot)

    def export_kml(self, filename, dataname, x_name, y_name):
        kml = kmlclass()
        kml.begin(filename, dataname, 'Description', 0.1)
        kml.trksegbegin('', '', 'red', 'absolute')

        for i in range(len(self.df[x_name])):
            kml.trkpt(self.df[x_name][i], self.df[y_name][i], 0.0)

        kml.trksegend()
        kml.end()

    def export_kml_data(self, filename, dataname, x_data, y_data):
        kml = kmlclass()
        kml.begin(filename, dataname, 'Description', 0.1)
        kml.trksegbegin('', '', 'red', 'absolute')

        for i in range(len(x_data)):
            kml.trkpt(x_data[i], y_data[i], 0.0)

        kml.trksegend()
        kml.end()

    def get_remove_outliers(self, points):
        lat_data = points[:, 0]
        lon_data = points[:, 1]
        utm_easting = []
        utm_northing = []
        geo_lat = []
        geo_lon = []
        variable_distance = 0

        # initialize variables
        uc = utmconv()
        old_e = 590686.5315567022
        old_n = 6136529.126779066
        # looping through file
        for i in range(len(lat_data)):

            lat = float(lat_data[i])
            lon = float(lon_data[i])

            (hemisphere, zone, letter, easting, northing) = uc.geodetic_to_utm(lat, lon)

            tmp_distance = sqrt((old_e - easting) ** 2 + (old_n - northing) ** 2)

            if tmp_distance < 1 + variable_distance: # and time < 527.542:
                utm_easting.append(easting)
                utm_northing.append(northing)

                geo_lat.append(lat)
                geo_lon.append(lon)
                variable_distance = 0
                old_e = easting
                old_n = northing
            else:
                variable_distance += 0.2

        # self.plot_data(utm_easting, utm_northing, 'Data', 'Easting', 'Northing', 'Position')

        points = np.column_stack((geo_lat, geo_lon))

        return points

    def perpendicular_dist(self, point, line_point1, line_point2):
        p1 = np.array(line_point1)
        p2 = np.array(line_point2)
        p3 = np.array(point)

        d = norm(np.cross(p2 - p1, p1 - p3)) / norm(p2 - p1)
        return d

    def get_number_points(self, points, nr_points):
        epsilon = 0.0000001
        current_number = nr_points + 1
        while current_number > nr_points:
            test_points = self.get_douglas_peucker(points, epsilon)
            current_number = len(test_points)
            epsilon = epsilon + 0.0000005

        return test_points

    def get_douglas_peucker(self, points, epsilon):
        # Find the point the maximum distance
        dmax = 0
        index = 0
        end = len(points) - 1
        for i in range(1, end - 1):
            d = self.perpendicular_dist(points[i], points[0], points[end])
            if d > dmax:
                index = i
                dmax = d

        # If max distance is greater than epsilon, recursively simplify
        if dmax > epsilon:
            # Recursive call
            rec_results1 = self.get_douglas_peucker(points[0:index], epsilon)
            rec_results2 = self.get_douglas_peucker(points[index:end], epsilon)

            # Build the result list
            results = np.vstack((rec_results1, rec_results2))
        else:
            results = np.vstack((points[0], points[end]))

        # Return the result
        return results

    def distance(self, p1, p2):
        uc = utmconv()
        (hemisphere, zone, letter, easting1, northing1) = uc.geodetic_to_utm(p1[0], p1[1])
        (hemisphere, zone, letter, easting2, northing2) = uc.geodetic_to_utm(p2[0], p2[1])
        return sqrt((easting1 - easting2)**2 + (northing1 - northing2)**2)

    def angle(self, p1, p2, p3):
        c = self.distance(p1, p2)
        b = self.distance(p1, p3)
        a = self.distance(p2, p3)
        angle = acos((c**2 + b**2 - a**2) / (2*c*b))
        return angle

    def get_remove_by_angle(self, points, angle):

        points_to_save = []
        ref_point = points[0]
        mid_point = points[1]

        points_to_save.append(ref_point)
        points_to_save.append(mid_point)

        for i in range(len(points) - 2):
            if i < 2:
                continue
            new_point = points[i]
            mid_point = points[i - 1]
            ref_point = points[i - 2]
            if angle < self.angle(ref_point, mid_point, new_point):
                points_to_save.append(new_point)

        points_to_save.append(points[len(points) - 1])

        return np.array(points_to_save)

    def Visvalingam_Whyatt(self, utm_easting, utm_northing):
        # Visvalingam-Whyatt algorithm
        x = utm_easting
        y = utm_northing
        var_area = 0
        cnt = 0
        while True:
            cnt += 1
            remove_list_e = []
            remove_list_n = []
            for i in range(len(utm_easting)):
                if i == 0:
                    continue
                if i == len(utm_easting) - 1:
                    break

                area = (0.5)
                abs((((x[i - 1] - x[i + 1])(y[i] - y[i - 1])) - ((x[i - 1] -
                                                                  x[i])(y[i + 1] - y[i - 1]))))

                print('area of triangle ' + str(area))
                print(x in i + str(x[i]))
                print(y in i + str(y[i]))

                if area > 0.04 + var_area:
                    remove_list_e.append(utm_easting[i])
                    remove_list_n.append(utm_northing[i])
            print(cnt + str(cnt))
            var_area += 0.005
            if (len(utm_easting) - len(remove_list_e)) > 33:
                break

        print('length of remove list ' + str(len(remove_list_e)))

        # prints removed points in remove_list
        # print(Points removed  + str(remove_list))

        print('length of list before removal ' + str(len(utm_easting)))
        for i in remove_list_e:  # range(len(utm_easting))
            if i in utm_easting:
                utm_easting.remove(i)

        for i in remove_list_n:
            if i in utm_northing:
                utm_northing.remove(i)

            # if(utm_easting[remove] == remove_list[0])
            # print(test)
            # print(utm_easting[remove])
            # del(utm_easting[remove])
        ##        utm_easting.remove(remove_list[1])
        #        remove_list.pop(remove)
        print('length of list after del ' + str(len(utm_easting)))
        return utm_easting, utm_northing

    def export_to_QGC(self, filename, points):
        plan = {}
        geoFence = {}
        plan['fileType'] = 'Plan'

        geoFence['polygon'] = []
        geoFence['version'] = 1
        plan['geoFence'] = geoFence

        plan['groundStation'] = 'QGroundControl'

        items = []

        item = {}
        item['autoContinue'] = True
        item['command'] = 22
        item['doJumpId'] = 1
        item['frame'] = 3
        item['params'] = [0, 0, 0, 0, 55.4713271, 10.32561629, 50]
        item['type'] = 'SimpleItem'
        items.append(item)

        item = {}
        item['autoContinue'] = True
        item['command'] = 16
        item['doJumpId'] = 2
        item['frame'] = 3
        item['params'] = [0, 0, 0, 0, 55.4714042, 10.3256544, 50]
        item['type'] = 'SimpleItem'
        items.append(item)

        item = {}
        item['autoContinue'] = True
        item['command'] = 16
        item['doJumpId'] = 3
        item['frame'] = 3
        item['params'] = [0, 0, 0, 0, 55.4710709, 10.3246252, 50]
        item['type'] = 'SimpleItem'
        items.append(item)

        item = {}
        item['autoContinue'] = True
        item['command'] = 16
        item['doJumpId'] = 4
        item['frame'] = 3
        item['params'] = [0, 0, 0, 0, 55.4708023, 10.3263024, 50]
        item['type'] = 'SimpleItem'
        items.append(item)

        mission = {}
        mission['cruiseSpeed'] = 15
        mission['firmwareType'] = 3
        mission['hoverSpeed'] = 5
        mission['items'] = items
        mission['plannedHomePosition'] = [55.4713, 10.3256, 50]
        mission['vehicleType'] = 2
        mission['version'] = 2
        plan['mission'] = mission

        arrayPoints = []
        for i in range(len(points)):
            arrayPoints.append(points[i, 0])
            arrayPoints.append(points[i, 1])

        rallyPoints = {}
        rallyPoints['points'] = arrayPoints
        rallyPoints['version'] = 1
        plan['rallyPoints'] = rallyPoints

        plan['version'] = 1

        plan_json = json.dumps(plan, indent=4, sort_keys=True)

        file = open(filename, 'w')
        file.write(plan_json)
        file.close()
