#include <Eigen/Dense>
#include "sr_ukf.h"
#include "spherical_conversions.h"

int main()
{
    Eigen::Vector<float, 9> initial {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    Eigen::Vector<float, 9> initial_stddevs {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    Eigen::Vector<float, 9> process_stddevs {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
    Eigen::Vector<float, 3> measurement_stddevs{0.1, 0.1, 0.1};
    g_point reference_pose = {43.084361, -77.6698107, 176};

    sr_ukf srukf(initial, initial_stddevs, process_stddevs, measurement_stddevs, reference_pose);

    srukf.predict(Eigen::Vector<float, 6>(0.1, 0.1, 0.1, 0.1, 0.1, 0.1), 0.1);
}
