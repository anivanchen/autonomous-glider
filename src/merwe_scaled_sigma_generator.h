#ifndef MERWE_SCALED_SIGMA_GENERATOR_
#define MERWE_SCALED_SIGMA_GENERATOR_

#define N 6

#include <Eigen/Dense>

class merwe_scaled_sigma_generator
{
public:
    merwe_scaled_sigma_generator();
    Eigen::Vector<float, 2 * N + 1>& wm();
    Eigen::Vector<float, 2 * N + 1>& wc();
    Eigen::Matrix<float, N, 2 * N + 1> square_root_sigma_points(const Eigen::Vector<float, N> &x, const Eigen::Matrix<float, N, N> &S);

private:
    Eigen::Vector<float, 2 * N + 1> m_wm;
    Eigen::Vector<float, 2 * N + 1> m_wc;
    float m_alpha;
    float m_kappa;

    void compute_weights(float beta);
};

#endif
