#include <Eigen/Dense>
#include "sr_ukf.h"
#include "spherical_conversions.h"

int main()
{
    // the way we should go about doing this is

    // wait for an output from either imu or gps

    // if imu measurement, run predict using dt since last run
    // save timestamp

    // if gps measurement, run predict using dt since last run
    // run update using dt since last run
    // save timestamp

    // this loop continues forever

    // sr_ukf.get_xhat() returns an Eigen::Vector containing
    // x y z vx vy vz yaw pitch roll as floats

    // if possible maybe run the kalman filter on a separate thread from the glider control logic?
    // not sure whether this is possible on these microcontrollers

    Eigen::Vector<float, 9> initial {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    Eigen::Vector<float, 9> initial_stddevs {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    Eigen::Vector<float, 9> process_stddevs {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
    Eigen::Vector<float, 3> measurement_stddevs{0.1, 0.1, 0.1};
    g_point reference_pose = {43.084361, -77.6698107, 176};

    sr_ukf srukf(initial, initial_stddevs, process_stddevs, measurement_stddevs, reference_pose);

    srukf.predict(Eigen::Vector<float, 6>(0.1, 0.1, 0.1, 0.1, 0.1, 0.1), 0.1);
}
