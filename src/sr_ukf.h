#ifndef SR_UKF_
#define SR_UKF_

#define N 9
#define INPUTS 6
#define ROWS 3

#include <Eigen/Dense>
#include "merwe_scaled_sigma_generator.h"
#include "spherical_conversions.h"

class sr_ukf {
    public:
    sr_ukf(const Eigen::Vector<float, N>& initial_state, const Eigen::Vector<float, N>& initial_stddevs, const Eigen::Vector<float, N>& process_stddevs, const Eigen::Vector<float, ROWS>& measurement_stddevs, const g_point& reference_pose);

    void predict(const Eigen::Vector<float, INPUTS>& u, const float& dt);
    void update(const Eigen::Vector<float, ROWS>& u, const float& dt);

    void set_contR(const Eigen::Vector<float, ROWS>& new_R);
    void set_xhat(const Eigen::Vector<float, N>& new_xhat);

    Eigen::Vector<float, N>& get_xhat();
    Eigen::Matrix<float, N, N>& get_s();

    Eigen::Vector<float, ROWS> h_gps(const Eigen::Vector<float, N>& x);

    private:
    Eigen::Vector<float, N> m_xhat;
    Eigen::Matrix<float, N, N> m_S;
    Eigen::Matrix<float, N, N> m_contQ;
    Eigen::Matrix<float, ROWS, ROWS> m_contR;
    Eigen::Matrix<float, N, 2 * N + 1> m_sigmasf;
    merwe_scaled_sigma_generator m_pts;
    c_point m_reference_pose;
    
    Eigen::Vector<float, N + INPUTS> process_model(const Eigen::Vector<float, N + INPUTS>& x);
    Eigen::Vector<float, N> rk4(const Eigen::Vector<float, N>& x, const Eigen::Vector<float, INPUTS>& u, const float& dt);
    
    void discretize_aq(const Eigen::Matrix<float, N, N>& cont_a, const Eigen::Matrix<float, N, N>& cont_q, float dt, Eigen::Matrix<float, N, N>* disc_a, Eigen::Matrix<float, N, N>* disc_q);
    Eigen::Matrix<float, 2 * N, 2 * N> mat_exp(const Eigen::Matrix<float, 2 * N, 2 * N>& A);

    float normalize_angle180(float x);
    float normalize_angle360(float x);

    Eigen::Vector<float, ROWS> measurement_mean(const Eigen::Matrix<float, ROWS, 2 * N + 1>& sigmas, const Eigen::Vector<float, 2 * N + 1>& wm);
    Eigen::Vector<float, ROWS> measurement_residual(const Eigen::Vector<float, ROWS>& a, const Eigen::Vector<float, ROWS>& b);
    Eigen::Vector<float, N> state_mean(const Eigen::Matrix<float, N, 2 * N + 1>& sigmas, const Eigen::Vector<float, 2 * N + 1>& wm);
    Eigen::Vector<float, N> state_residual(const Eigen::Vector<float, N> a, const Eigen::Vector<float, N>& b);
    Eigen::Vector<float, N> state_add(const Eigen::Vector<float, N>& a, const Eigen::Vector<float, N>& b);
};

#endif
