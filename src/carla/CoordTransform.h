#ifndef CARLA_COORDTRANSFORM_H_
#define CARLA_COORDTRANSFORM_H_

#include <cmath>

namespace artery
{
namespace carla
{

/**
 * A pose in a single convention pair.
 *
 * OpenDRIVE/SUMO: right handed, x east, y north, heading in *degrees*
 * counter-clockwise from +x (math convention; OpenDRIVE stores radians).
 * CARLA:          left handed, x/y/z as produced by the server.
 *
 * The relation (the same one CARLA's own importers use):
 *
 *     x_carla =  x_xodr
 *     y_carla = -y_xodr
 *     z_carla =  z_xodr
 *     yaw_carla(deg) = -hdg_xodr(deg)
 *
 * SUMO/TraCI angles are degrees *clockwise from north* (north = +y of the
 * xodr/SUMO plane):
 *
 *     hdg_xodr(deg) = 90 - angle_sumo(deg)
 *     yaw_carla(deg) = angle_sumo(deg) - 90
 */
struct CarlaPose
{
    double x {0.0};
    double y {0.0};
    double z {0.0};
    double yaw {0.0};  ///< degrees, CARLA convention
};

inline CarlaPose xodrToCarla(double x, double y, double z, double yaw_xodr_deg)
{
    return CarlaPose { x, -y, z, -yaw_xodr_deg };
}

inline void carlaToXodr(const CarlaPose& pose, double& x, double& y, double& z,
    double& yaw_xodr_deg)
{
    x = pose.x;
    y = -pose.y;
    z = pose.z;
    yaw_xodr_deg = -pose.yaw;
}

inline double xodrYawDegToSumoDeg(double hdg_xodr_deg)
{
    return 90.0 - hdg_xodr_deg;
}

inline double sumoDegToXodrYawDeg(double angle_sumo_deg)
{
    return 90.0 - angle_sumo_deg;
}

inline double sumoDegToCarlaYawDeg(double angle_sumo_deg)
{
    return angle_sumo_deg - 90.0;
}

/// Full SUMO (x, y, angle-degrees-from-north-CW) -> CARLA pose transform.
inline CarlaPose sumoToCarla(double x, double y, double z, double angle_sumo_deg)
{
    return CarlaPose { x, -y, z, sumoDegToCarlaYawDeg(angle_sumo_deg) };
}

/// Euclidean distance between two CARLA positions (pose ignored beyond x/y/z).
inline double positionDistance(const CarlaPose& a, const CarlaPose& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace carla
}  // namespace artery

#endif
