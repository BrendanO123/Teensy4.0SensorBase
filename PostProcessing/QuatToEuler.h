#ifndef QUAT_TO_EULER_H
#define QUAT_TO_EULER_H

#include <cmath>

const double PI = 3.14159265359;

struct EulerAngles {
    double roll;
    double pitch;
    double yaw;
};

inline EulerAngles quaternionToEuler(double q1, double q2, double q3) {
    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
    
    double qw = q0;
    double qx = q2;
    double qy = q1;
    double qz = -q3;
    
    // roll (x-axis rotation)
    double t0 = +2.0 * (qw * qx + qy * qz);
    double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
    double roll = atan2(t0, t1) * 180.0 / PI;
    
    // pitch (y-axis rotation)
    double t2 = +2.0 * (qw * qy - qx * qz);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    double pitch = asin(t2) * 180.0 / PI;
    
    // yaw (z-axis rotation)
    double t3 = +2.0 * (qw * qz + qx * qy);
    double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
    double yaw = atan2(t3, t4) * 180.0 / PI;
    
    return {roll, pitch, yaw};
}

#endif
