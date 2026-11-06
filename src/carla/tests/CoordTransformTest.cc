// Unit test for the SUMO/OpenDRIVE <-> CARLA coordinate transform.
//
// Build & run:  cmake --build build --target carla_coordtest && ./build/src/carla/carla_coordtest
// Registered as a CTest test ("coordtransform") as well.

#include "carla/CoordTransform.h"

#include <cmath>
#include <cstdio>

using artery::carla::CarlaPose;
using artery::carla::carlaToXodr;
using artery::carla::sumoDegToCarlaYawDeg;
using artery::carla::sumoDegToXodrYawDeg;
using artery::carla::sumoToCarla;
using artery::carla::xodrToCarla;
using artery::carla::xodrYawDegToSumoDeg;

namespace
{

int failures = 0;

void checkNear(double actual, double expected, const char* what)
{
    if (std::fabs(actual - expected) > 1e-9) {
        std::printf("FAIL %s: got %.9f expected %.9f\n", what, actual, expected);
        ++failures;
    }
}

void check(bool condition, const char* what)
{
    if (!condition) {
        std::printf("FAIL %s\n", what);
        ++failures;
    }
}

}  // namespace

int main()
{
    // --- handedness: a point 10 m north (x=0,y=10) is -y in CARLA ---------
    const CarlaPose north = xodrToCarla(0.0, 10.0, 0.0, 90.0);
    checkNear(north.x, 0.0, "north.x");
    checkNear(north.y, -10.0, "north.y");
    checkNear(north.yaw, -90.0, "north.yaw");

    // --- round trip --------------------------------------------------------
    double x = 0, y = 0, z = 0, yaw = 0;
    carlaToXodr(north, x, y, z, yaw);
    checkNear(x, 0.0, "roundtrip.x");
    checkNear(y, 10.0, "roundtrip.y");
    checkNear(yaw, 90.0, "roundtrip.yaw");

    // --- SUMO angle conventions -------------------------------------------
    // SUMO angle 0 = north (+y in the xodr plane, math heading +90 deg)
    checkNear(sumoDegToXodrYawDeg(0.0), 90.0, "sumo0->xodr");
    checkNear(sumoDegToXodrYawDeg(90.0), 0.0, "sumo90(east)->xodr 0");
    checkNear(xodrYawDegToSumoDeg(90.0), 0.0, "xodr90->sumo");
    checkNear(sumoDegToCarlaYawDeg(0.0), -90.0, "sumo0->carla north");
    checkNear(sumoDegToCarlaYawDeg(90.0), 0.0, "sumo90->carla east");

    // --- heading consistency: the heading vector must transform with the ---
    // --- point.  Moving north in SUMO for 1 s at 1 m/s:                    //
    // p0=(0,0), p1=(0,1); in CARLA p1'=(0,-1), i.e. yaw=-90 must point to   //
    // -y_carla.  cos/sin of the CARLA yaw gives the travel direction in the //
    // CARLA plane.                                                           //
    const CarlaPose pose = sumoToCarla(0.0, 0.0, 0.0, 0.0);
    const double dir_x = std::cos(pose.yaw * M_PI / 180.0);
    const double dir_y = std::sin(pose.yaw * M_PI / 180.0);
    check(dir_y < -0.99 && std::fabs(dir_x) < 0.01, "carla yaw -90 points to -y");

    // same for eastbound (SUMO angle 90 -> +x in both worlds)
    const CarlaPose east = sumoToCarla(0.0, 0.0, 0.0, 90.0);
    checkNear(std::cos(east.yaw * M_PI / 180.0), 1.0, "carla yaw 0 points to +x");

    // --- consistency of the two conversions --------------------------------
    for (double a = -720; a <= 720; a += 7.5) {
        const double direct = sumoDegToCarlaYawDeg(a);
        const double indirect = -sumoDegToXodrYawDeg(a);
        checkNear(direct, indirect, "sumo->carla consistency");
    }

    if (failures == 0) {
        std::printf("CoordTransform tests: OK\n");
        return 0;
    }
    std::printf("%d failure(s)\n", failures);
    return 1;
}
