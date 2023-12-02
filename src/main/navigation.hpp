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
    double a = hav(static_cast<double>(lat2 - lat1) / 1E6);
    double b = cos(lat1 * M_PI / 180E6) * cos(lat2 * M_PI / 180E6);
    double c = hav(static_cast<double>(long2 - long1) / 1E6);
    return static_cast<float>(R_EARTH_M * 2.0 * atan2(sqrt(a + (b*c)), sqrt(1.0 - (a + (b*c)))));
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
    double x = cos(static_cast<double>(lat2) * M_PI / 180E6) * sin(static_cast<double>(long2 - long1) * M_PI / 180E6);
    double y1 = cos(static_cast<double>(lat1) * M_PI / 180E6) * sin(static_cast<double>(lat2) * M_PI / 180E6); 
    double y2 = sin(static_cast<double>(lat1) * M_PI / 180E6) * cos(static_cast<double>(lat2) * M_PI / 180E6) * cos(static_cast<double>(long2 - long1) * M_PI / 180E6);
    double brng = atan2(x, y1-y2);
    return static_cast<double>(brng * 180.0 / M_PI);
}

}