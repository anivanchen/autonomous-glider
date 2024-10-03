#ifndef SR_UKF_
#define SR_UKF_

#define N 9
#define INPUTS 6
#define ROWS 3

#include <Eigen/Dense>
#include "merwe_scaled_sigma_generator.h"

class sr_ukf {
    public:
    sr_ukf(Eigen::Vector<float, N> initial_state, Eigen::Vector<float, N> initial_stddevs, Eigen::Vector<float, N> process_stddevs, Eigen::Vector<float, ROWS> measurement_stddevs);

    void set_contR(Eigen::Vector<float, ROWS> new_R);
    void update(Eigen::Vector<float, ROWS> u, float dt);
    void predict(Eigen::Vector<float, INPUTS> u, float dt);

    void discretize_aq(const Eigen::Matrix<float, N, N>& cont_a, const Eigen::Matrix<float, N, N>& cont_q, float dt, Eigen::Matrix<float, N, N>* disc_a, Eigen::Matrix<float, N, N>* disc_q);
    Eigen::Matrix<float, 2 * N, 2 * N> mat_exp(const Eigen::Matrix<float, 2 * N, 2 * N>& A);

    Eigen::Vector<float, N> get_xhat();
    void set_xhat(Eigen::Vector<float, N> new_xhat);
    Eigen::Matrix<float, N, N> get_s();

    Eigen::Vector<float, ROWS> h_gps(Eigen::Vector<float, N> x);

    private:
    Eigen::Vector<float, N> m_xhat;
    Eigen::Matrix<float, N, N> m_S;
    Eigen::Matrix<float, N, N> m_contQ;
    Eigen::Matrix<float, ROWS, ROWS> m_contR;
    Eigen::Matrix<float, N, 2 * N + 1> m_sigmasf;
    merwe_scaled_sigma_generator m_pts;


    
    Eigen::Vector<float, N + INPUTS> process_model(Eigen::Vector<float, N + INPUTS> x);
    Eigen::Vector<float, N> rk4(Eigen::Vector<float, N> x, Eigen::Vector<float, INPUTS> u, float dt);

    
    Eigen::Vector<float, ROWS> measurement_mean(Eigen::Matrix<float, ROWS, 2 * N + 1> sigmas, Eigen::Vector<float, 2 * N + 1> wm);
    Eigen::Vector<float, ROWS> measurement_residual(Eigen::Vector<float, ROWS> a, Eigen::Vector<float, ROWS> b);
    

    float normalize_angle180(float x);
    float normalize_angle360(float x);
    Eigen::Vector<float, N> state_mean(Eigen::Matrix<float, N, 2 * N + 1> sigmas, Eigen::Vector<float, 2 * N + 1> wm);
    Eigen::Vector<float, N> state_residual(Eigen::Vector<float, N> a, Eigen::Vector<float, N> b);
    Eigen::Vector<float, N> state_add(Eigen::Vector<float, N> a, Eigen::Vector<float, N> b);

};

#endif
