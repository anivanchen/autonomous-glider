#include <Eigen/Dense>
#include "sr_ukf.h"
#include "spherical_conversions.h"

sr_ukf::sr_ukf(const Eigen::Vector<float, N>& initial_state, const Eigen::Vector<float, N>& initial_stddevs, const Eigen::Vector<float, N>& process_stddevs, const Eigen::Vector<float, ROWS>& measurement_stddevs, const g_point& reference_pose) {
    m_xhat = initial_state;

    // these are intended to be the square root of what they would usually be
    m_contQ = process_stddevs.asDiagonal();
    m_contR = measurement_stddevs.asDiagonal();
    m_S = initial_stddevs.asDiagonal();    

    m_reference_pose = geodetic_to_ecef(reference_pose);
}

void sr_ukf::predict(const Eigen::Vector<float, INPUTS>& u, const float& dt) {
    // discretize the noise model
    Eigen::Matrix<float, N, N> disc_q = m_contQ * sqrt(dt);


    // generate sigma points
    Eigen::Matrix<float, N, 2 * N + 1> sigmas = m_pts.square_root_sigma_points(m_xhat, m_S);
    m_sigmasf.setZero();

    // project sigmas through process model (merwe 18)
    for (int i = 0; i < (2 * N + 1); i++) {
        m_sigmasf.col(i) = (rk4(sigmas.col(i), u, dt));
    }

    // xhat is mean of sigmas (merwe 19)
    m_xhat = state_mean(m_sigmasf, m_pts.wm());

    // qr of matrix of weighted sigmas, and sqrt(noise cov) (merwe 20)
    Eigen::Vector<float, 2 * N + 1> wc = m_pts.wc();
    Eigen::Matrix<float, N, N * 2 + N> sbar;
    for (int i = 0; i < N * 2; i++) {
        sbar.col(i) = std::sqrt(wc(1)) * state_residual(m_sigmasf.col(1 + i), m_xhat);
    }
    sbar.block<N, N>(0, N * 2) = disc_q.triangularView<Eigen::Lower>();
    Eigen::Matrix<float, N, N> S = sbar.transpose().householderQr().matrixQR().block<N, N>(0, 0).triangularView<Eigen::Upper>();

    // cholupdate (merwe 21)
    Eigen::internal::llt_inplace<float, Eigen::Upper>::rankUpdate(S, state_residual(m_sigmasf.col(0), m_xhat), wc(0));
    m_S = S;
}

void sr_ukf::update(const Eigen::Vector<float, ROWS>& y, const float& dt) {
    // discretize the noise model
    Eigen::Matrix<float, ROWS, ROWS> disc_r = m_contR * sqrt(dt);

    // project sigmas through measurement function, they become predicted measurements at their state (merwe 22)
    Eigen::Matrix<float, ROWS, 2 * N + 1> h_sigmas;
    Eigen::Matrix<float, N, 2 * N + 1> sigmas;
    sigmas = m_pts.square_root_sigma_points(m_xhat, m_S);

    for (int i = 0; i < (2 * N + 1); i++) {
        h_sigmas.col(i) = h_gps(m_sigmasf.col(i));
    }

    // yhat is mean of measurement sigmas (merwe 23)
    Eigen::Vector<float, ROWS> yhat = measurement_mean(h_sigmas, m_pts.wm());

    // qr of matrix of weighted measurement sigmas, and sqrt(noise cov) (merwe 24)
    Eigen::Vector<float, 2 * N + 1> wc = m_pts.wc();
    Eigen::Matrix<float, ROWS, N * 2 + ROWS> sbar;
    for (int i = 0; i < N * 2; i++) {
        sbar.col(i) = std::sqrt(wc(1)) * measurement_residual(h_sigmas.col(1 + i), yhat);
    }
    sbar.block<ROWS, ROWS>(0, N * 2) = disc_r.triangularView<Eigen::Lower>();
    Eigen::Matrix<float, ROWS, ROWS> sy = sbar.transpose().householderQr().matrixQR().block<ROWS, ROWS>(0, 0).triangularView<Eigen::Upper>();

    // cholupdate (merwe 25)
    Eigen::internal::llt_inplace<float, Eigen::Upper>::rankUpdate(sy, measurement_residual(h_sigmas.col(0), yhat), wc(0));

    // pxy (merwe 26)
    Eigen::Matrix<float, N, ROWS> pxy;
    pxy.fill(0);
    for (int i = 0; i < (2 * N + 1); i++) {
        pxy += wc(i) * (state_residual(m_sigmasf.col(i), m_xhat) * measurement_residual(h_sigmas.col(i), yhat).transpose());
    }

    // calculate kalman gain with two least squares solutions (merwe 26)
    Eigen::Matrix<float, N, ROWS> K = sy.transpose().fullPivHouseholderQr().solve(sy.fullPivHouseholderQr().solve(pxy.transpose())).transpose();

    // update mean using kalman gain and measurement residual (merwe 27)
    m_xhat = state_add(m_xhat, K * measurement_residual(y, yhat));

    // intermediate matrix to find new covariance (merwe 28)
    Eigen::Matrix<float, N, ROWS> U = K * sy;

    // cholupdate covariance (merwe 29)
    for (int i = 0; i < ROWS; i++) {
        Eigen::internal::llt_inplace<float, Eigen::Upper>::rankUpdate(m_S, U.col(i), -1);
    }
}

void sr_ukf::set_contR(const Eigen::Vector<float, ROWS>& new_R) {
    m_contR = new_R.asDiagonal();
}

void sr_ukf::set_xhat(const Eigen::Vector<float, N>& new_xhat) {
    m_xhat = new_xhat;
}

Eigen::Vector<float, N>& sr_ukf::get_xhat() {
    return m_xhat;
}

Eigen::Matrix<float, N, N>& sr_ukf::get_s() {
    return m_S;
}

Eigen::Vector<float, ROWS> sr_ukf::h_gps(const Eigen::Vector<float, N>& x) {
    c_point pose = {x(0), x(1), x(2)};
    g_point g_pose = enu_to_geodetic(m_reference_pose, pose);
    Eigen::Vector<float, ROWS> out = {(float) g_pose.lat, (float) g_pose.lon, (float) g_pose.height};

    return out;
}

// private methods

Eigen::Vector<float, N + INPUTS> sr_ukf::process_model(const Eigen::Vector<float, N + INPUTS>& in) {
    float xdotdot = in(9);
    float ydotdot = in(10);
    float zdotdot = in(11);
    float xdot = in(3);
    float ydot = in(4);
    float zdot = in(5);


    return Eigen::Vector<float, N + INPUTS>{xdot, ydot, zdot, xdotdot, ydotdot, zdotdot, 0, 0, 0};
}

Eigen::Vector<float, N> sr_ukf::rk4(const Eigen::Vector<float, N>& x, const Eigen::Vector<float, INPUTS>& u, const float& dt) {
    float h = dt;
    // concatenate x and u vectors so rk4 can operate on it as one vector
    Eigen::Vector<float, N + INPUTS> x_u;
    x_u << x, u;

    Eigen::Vector<float, N + INPUTS> k_1 = process_model(x_u);
    Eigen::Vector<float, N + INPUTS> k_2 = process_model(x_u + (k_1 * (h / 2)));
    Eigen::Vector<float, N + INPUTS> k_3 = process_model(x_u + (k_2 * (h / 2)));
    Eigen::Vector<float, N + INPUTS> k_4 = process_model(x_u + (k_3 * h));

    // remove u from each vector
    Eigen::Vector<float, N> xk_1 = k_1.block<N, 1>(0, 0);
    Eigen::Vector<float, N> xk_2 = k_2.block<N, 1>(0, 0);
    Eigen::Vector<float, N> xk_3 = k_3.block<N, 1>(0, 0);
    Eigen::Vector<float, N> xk_4 = k_4.block<N, 1>(0, 0);

    return (x + ((h / 6) * (xk_1 + (2 * xk_2) + (2 * xk_3) + xk_4)));
}

float sr_ukf::normalize_angle180(float x) {
    x = fmod(x + M_PI, 2 * M_PI);
    if (x < 0) {
        x += (2 * M_PI);
    }
    return x - M_PI;
}

float sr_ukf::normalize_angle360(float x) {
    x = fmod(x, 2 * M_PI);
    if (x < 0) {
        x += (2 * M_PI);
    }
    return x;
}

Eigen::Vector<float, ROWS> sr_ukf::measurement_mean(const Eigen::Matrix<float, ROWS, 2 * N + 1>& sigmas, const Eigen::Vector<float, 2 * N + 1>& wm) {
    Eigen::Vector<float, ROWS> x;
    x.fill(0);

    float latsum_sin = 0;
    float latsum_cos = 0;
    float lonsum_sin = 0;
    float lonsum_cos = 0;

    for (int i = 0; i < (2 * N + 1); i++) {
        // assume r,theta..... rewrite for glider (will be easier)
        x(0) += sigmas(0, i) * wm(i);
        latsum_sin += std::sin(sigmas(1, i)) * wm(i);
        latsum_cos += std::cos(sigmas(1, i)) * wm(i);
        lonsum_sin += std::sin(sigmas(1, i)) * wm(i);
        lonsum_cos += std::cos(sigmas(1, i)) * wm(i);
    }

    x(1) = std::atan2(latsum_sin, latsum_cos);
    x(2) = std::atan2(lonsum_sin, lonsum_cos);
    return x;
}

Eigen::Vector<float, ROWS> sr_ukf::measurement_residual(const Eigen::Vector<float, ROWS>& a, const Eigen::Vector<float, ROWS>& b) {
    Eigen::Vector<float, ROWS> out = a - b;
    out(2) = normalize_angle180(out(2));

    return out;
}

Eigen::Vector<float, N> sr_ukf::state_mean(const Eigen::Matrix<float, N, 2 * N + 1>& sigmas, const Eigen::Vector<float, 2 * N + 1>& wm) {
    Eigen::Vector<float, N> x;



    for (int i = 0; i < (2 * N + 1); i++) {
        // assume x,y,theta...... rewrite for glider :sob:
        x(0) += sigmas(0, i) * wm(i);
        x(1) += sigmas(1, i) * wm(i);
        x(2) += sigmas(2, i) * wm(i);
        x(3) += sigmas(3, i) * wm(i);
        x(4) += sigmas(4, i) * wm(i);
        x(5) += sigmas(5, i) * wm(i);

    }

    return x;
}

Eigen::Vector<float, N> sr_ukf::state_residual(const Eigen::Vector<float, N> a, const Eigen::Vector<float, N>& b) {
    Eigen::Vector<float, N> out = a - b;


    return out;
}

Eigen::Vector<float, N> sr_ukf::state_add(const Eigen::Vector<float, N>& a, const Eigen::Vector<float, N>& b) {
    Eigen::Vector<float, N> out = a + b;
    out(6) = normalize_angle180(out(6));
    out(7) = normalize_angle180(out(7));
    out(8) = normalize_angle180(out(8));

    return out;
}
