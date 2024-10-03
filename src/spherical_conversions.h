#include <Eigen/Dense>

Eigen::Vector<float, 3> enu_to_ecef(Eigen::Vector<double, 3> reference_ecef, Eigen::Vector<double, 3> point_enu);