from utm import utmconv
from math import acos, asin, sqrt, sin, cos, pi


def distance_between_points(lat1, lon1, lat2, lon2):
    lat1 = lat1 * pi / 180
    lat2 = lat2 * pi / 180
    lon1 = lon1 * pi / 180
    lon2 = lon2 * pi / 180

    distance_radians = acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(lon1 - lon2))
    distance_km = distance_radians * 6371.0
    return distance_km


lat_ref = 55.47
lon_ref = 10.33

uc = utmconv()

(hemisphere, zone, letter, easting, northing) = uc.geodetic_to_utm(lat_ref, lon_ref)
print('\nReference point in UTM:')
print('  %d %c %.5fe %.5fn' % (zone, letter, easting, northing))

easting_2 = easting + 1000
(lat_east, lon_east) = uc.utm_to_geodetic (hemisphere, zone, easting_2, northing)
print('\nConverted back from UTM to geodetic [deg] for 1 km east:')
print('  latitude:  %.10f' % lat_east)
print('  longitude: %.10f' % lon_east)

northing_2 = northing + 1000
(lat_north, lon_north) = uc.utm_to_geodetic (hemisphere, zone, easting, northing_2)
print('\nConverted back from UTM to geodetic [deg] for 1 km north:')
print('  latitude:  %.10f' % lat_north)
print('  longitude: %.10f' % lon_north)

distance = distance_between_points(lat_ref, lon_ref, lat_east, lon_east)
print('\nDistance between reference point and eastern point:')
print(distance)
print('\nError:')
print(1 - distance)

distance = distance_between_points(lat_ref, lon_ref, lat_north, lon_north)
print('\nDistance between reference point and northern point:')
print(distance)
print('\nError:')
print(1 - distance)


""" Stuff """

lat_ref = 55.47 * pi / 180
lon_ref = 10.33 * pi / 180
south_lat = pi*55.4618345/180
south_lon = pi*10.3305817/180


#Find distance between two points in geodetic coordinates
d_geodetic = 2*asin(sqrt((sin((lat_ref-south_lat)/2))**2 +cos(lat_ref)*cos(south_lat)*(sin((lon_ref-south_lon)/2))**2))
#THis distance is in radians, so now it needs to be first converted to nautical miles and then converted to kilometers
distance_nm = ((180*60)/pi)*d_geodetic
d = distance_nm*1.852
print("\nGeodetic distance between points aligned on longtitide: ", d)