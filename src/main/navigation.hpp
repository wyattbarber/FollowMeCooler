#define _USE_MATH_DEFINES
#include <cmath>

#define R_EARTH_M 6378100.0 //! Radius of the earth, in meters

namespace nav
{
/** Haversine function
 * 
 * Implements hav(theta) = sin^2(theta / 2)
 * 
 * @param theta argument, in degrees, to the haversine function
 * @return hav(theta) 
*/
double hav(double theta)
{
    double tr = theta * M_PI / 180.0;
    double s = sin(tr / 2.0);
    return pow(s, 2.0);
}


/** Distance between 2 GPS coordinates.
 * 
 * Calculates the distance between 2 points, with all coordinates 
 * given in millionths of degrees. 
 * 
 * @param lat1 latitude of point 1
 * @param lat2 latitude of point 2
 * @param long1 longitude of point 1
 * @param long2 longitude of point 2
 * 
 * @return Distance between the two points, in m
*/
float dist(long lat1, long lat2, long long1, long long2)
{
    float dlat = static_cast<float>(lat2 - lat1) * M_PI * (R_EARTH_M / 180E6);
    float dlon = static_cast<float>(long2 - long1) * M_PI * (R_EARTH_M / 180E6);
    return sqrt(pow(dlat, 2.0) + pow(dlon, 2.0));
}


/** Orientation of two GPS coordinates
 * 
 * Calculates the angle of one coordinate pair relative to another.
 * All coordinates given in millionths of degrees. 
 * 
 * The angle calculated as the angle from south from point 2 towards point 1.
 * Westward angles from point 2 are positive.
 * 
 * So, if point 2 is due north of point 1, the angle is 0 degrees.
 * If point 2 is due west of point 1, the angle is -90 degrees.
 * 
 * @param lat1 latitude of point 1
 * @param lat2 latitude of point 2
 * @param long1 longitude of point 1
 * @param long2 longitude of point 2
 * 
 * @return Angle between the two points, in degrees.
*/
float angle(long lat1, long lat2, long long1, long long2)
{
    float dlat = static_cast<float>(lat2 - lat1);
    float dlon = static_cast<float>(long2 - long1);
    return atan2(-dlon, dlat) * 180.0 / M_PI;
}

}