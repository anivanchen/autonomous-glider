#include <Eigen/Dense>
#include "sr_ukf.h"

int main()
{
    Eigen::Vector<float, 9> initial {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    Eigen::Vector<float, 9> initial_stddevs {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    Eigen::Vector<float, 9> process_stddevs {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
    Eigen::Vector<float, 3> measurement_stddevs{0.1, 0.1, 0.1};
    sr_ukf srukf(initial, initial_stddevs, process_stddevs, measurement_stddevs);

    srukf.predict(Eigen::Vector<float, 6>(0.1, 0.1, 0.1, 0.1, 0.1, 0.1), 0.1);
}
