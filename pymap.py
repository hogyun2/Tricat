import pymap3d as pm

def get_xy(base_lat,base_lon,base_alt,lat,lon,alt):

    e, n, u = pm.geodetic2enu(lat, lon, alt, base_lat, base_lon, base_alt)
    waypoint = (e,n,u)
    return waypoint

print(get_xy(37.4505348,126.6529716,0,37.4503229,126.6533548,0))
