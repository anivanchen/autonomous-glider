#include <Eigen/Dense>
#include "merwe_scaled_sigma_generator.h"

Eigen::Vector<float, 2 * N + 1>& merwe_scaled_sigma_generator::wm() {return m_wm;}
Eigen::Vector<float, 2 * N + 1>& merwe_scaled_sigma_generator::wc() {return m_wc;}

merwe_scaled_sigma_generator::merwe_scaled_sigma_generator()
{
    m_alpha = 1e-2;
    m_kappa = 0;
    float beta = 2;

    m_wm.resize(2 * N + 1);
    m_wc.resize(2 * N + 1);

    compute_weights(beta);
}

Eigen::Matrix<float, N, 2 * N + 1> merwe_scaled_sigma_generator::square_root_sigma_points(
    const Eigen::Vector<float, N>& x,
    const Eigen::Matrix<float, N, N>& S)
{
    const float lambda = (m_alpha * m_alpha) * (N + m_kappa) - N;
    float eta = std::sqrt(lambda + N);
    Eigen::Matrix<float, N, N> U;
    U = eta * S;

    Eigen::Matrix<float, N, 2 * N + 1> sigmas;
    sigmas.col(0) = x;
    for (int k = 0; k < N; k++)
    {
        sigmas.col(k + 1) = x + U.col(k);
        sigmas.col(N + k + 1) = x - U.col(k);
    }

    return sigmas;
}

void merwe_scaled_sigma_generator::compute_weights(float beta)
{
    const float lambda = (m_alpha * m_alpha) * (N + m_kappa) - N;
    const float c = 0.5 / (N + lambda);

    m_wm.fill(c);
    m_wc.fill(c);
    m_wc[0] = lambda / (N + lambda) + (1 - (m_alpha * m_alpha) + beta);
    m_wm[0] = lambda / (N + lambda);
}
