#include <Eigen/Dense>
#include "sr_ukf.h"

sr_ukf::sr_ukf(Eigen::Vector<float, N> initial_state, Eigen::Vector<float, N> initial_stddevs, Eigen::Vector<float, N> process_stddevs, Eigen::Vector<float, ROWS> measurement_stddevs) {
    m_xhat = initial_state;

    // these are intended to be the square root of what they would usually be
    m_contQ = process_stddevs.asDiagonal();
    m_contR = measurement_stddevs.asDiagonal();
    m_S = initial_stddevs.asDiagonal();    
}

void sr_ukf::set_contR(Eigen::Vector<float, ROWS> new_R) {
    m_contR = new_R.asDiagonal();
}

void sr_ukf::predict(Eigen::Vector<float, INPUTS> u, float dt) {
    // discretize the noise model
    Eigen::Matrix<float, N, N> cont_a;
    cont_a.setZero();
    for (int i = 0; i < N; ++i) {
        Eigen::Vector<float, N> dx_plus = m_xhat;
        dx_plus(i) += 1e-5;
        Eigen::Vector<float, N> dx_minus = m_xhat;
        dx_minus(i) -= 1e-5;
        cont_a.col(i) = (rk4(dx_plus, u, dt) - rk4(dx_minus, u, dt)) / (1e-5 * 2);
    }
    Eigen::Matrix<float, N, N> disc_a;
    Eigen::Matrix<float, N, N> disc_q;

    discretize_aq(cont_a, m_contQ, dt, &disc_a, &disc_q);
    Eigen::internal::llt_inplace<float, Eigen::Lower>::blocked(disc_q);


    // generate sigma points
    Eigen::Matrix<float, N, 2 * N + 1> sigmas = m_pts.square_root_sigma_points(m_xhat, m_S);

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




void sr_ukf::update(Eigen::Vector<float, ROWS> y, float dt) {
    // discretize the noise model
    Eigen::Matrix<float, ROWS, ROWS> disc_r = m_contR / std::sqrt(dt);

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




void sr_ukf::discretize_aq(const Eigen::Matrix<float, N, N>& cont_a, const Eigen::Matrix<float, N, N>& cont_q, float dt, Eigen::Matrix<float, N, N>* disc_a, Eigen::Matrix<float, N, N>* disc_q) {
    Eigen::Matrix<float, N, N> Q = (cont_q * cont_q.transpose());

    Eigen::Matrix<float, 2 * N, 2 * N> M;
    M.block<N, N>(0, 0) = -cont_a;
    M.block<N, N>(0, N) = Q;
    M.block<N, N>(N, 0).setZero();
    M.block<N, N>(N, N) = cont_a.transpose();

    Eigen::Matrix<float, 2 * N, 2 * N> phi = mat_exp(M * dt);

    Eigen::Matrix<float, N, N> phi12 = phi.block(0, N, N, N);

    Eigen::Matrix<float, N, N> phi22 = phi.block(N, N, N, N);

    *disc_a = phi22.transpose();

    Q = *disc_a * phi12;

    Eigen::LLT<Eigen::Matrix<float, N, N>> llt_of_q(Q);
    *disc_q = llt_of_q.matrixL();
}

Eigen::Matrix<float, 2 * N, 2 * N> sr_ukf::mat_exp(const Eigen::Matrix<float, 2 * N, 2 * N>& A) {    
    int n = A.rows(); // size of the matrix
    int s = 0;        // scaling factor for the matrix

    // compute the norm of the matrix (1-norm, which is the maximum absolute column sum)
    double normA = A.cwiseAbs().colwise().sum().maxCoeff();

    // if norm is too big scale down
    if (normA > 0.5) {
        s = std::max(0, static_cast<int>(std::ceil(std::log2(normA / 0.5))));
    }

    // scale the matrix by 2^(-s)
    Eigen::Matrix<float, 2 * N, 2 * N> A_scaled = A / std::pow(2.0, s);

    // compute [7/7] pade approximant for the scaled matrix
    Eigen::Matrix<float, 2 * N, 2 * N> X = A_scaled;
    double c = 0.5;
    Eigen::Matrix<float, 2 * N, 2 * N> E = Eigen::Matrix<float, 2 * N, 2 * N>::Identity(n, n) + c * A_scaled;
    Eigen::Matrix<float, 2 * N, 2 * N> D = Eigen::Matrix<float, 2 * N, 2 * N>::Identity(n, n) - c * A_scaled;
    bool positive = true;

    for (int k = 2; k <= 7; ++k) {
        c = c * (7 - k + 1) / (k * (2 * 7 - k + 1));
        X = A_scaled * X;
        Eigen::Matrix<float, 2 * N, 2 * N> cX = c * X;
        E = E + (positive ? cX : -cX);
        D = D + (positive ? cX : -cX);
        positive = !positive;
    }

    // solve D * expA = E to get the matrix exponential
    Eigen::Matrix<float, 2 * N, 2 * N> expA = D.lu().solve(E);

    // scale back up by squaring matrix
    for (int i = 0; i < s; ++i) {
        expA = expA * expA;
    }

    return expA;
}

Eigen::Vector<float, N> sr_ukf::get_xhat() {
    return m_xhat;
}

void sr_ukf::set_xhat(Eigen::Vector<float, N> new_xhat) {
    m_xhat = new_xhat;
}

Eigen::Matrix<float, N, N> sr_ukf::get_s() {
    return m_S;
}

Eigen::Vector<float, ROWS> sr_ukf::h_gps(Eigen::Vector<float, N> x) {

    

    return Eigen::Vector<float, ROWS>(1, 1, 1);
}

Eigen::Vector<float, N + INPUTS> sr_ukf::process_model(Eigen::Vector<float, N + INPUTS> in) {
    float xdotdot = in(9);
    float ydotdot = in(10);
    float zdotdot = in(11);
    float xdot = in(3);
    float ydot = in(4);
    float zdot = in(5);
    float psidot = in(12);
    float thetadot = in(13);
    float phidot = in(14);

    return Eigen::Vector<float, N + INPUTS>{xdot, ydot, zdot, xdotdot, ydotdot, zdotdot, psidot, thetadot, phidot, 0, 0, 0, 0, 0, 0};
}

Eigen::Vector<float, N> sr_ukf::rk4(Eigen::Vector<float, N> x, Eigen::Vector<float, INPUTS> u, float dt) {
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

Eigen::Vector<float, N> sr_ukf::state_mean(Eigen::Matrix<float, N, 2 * N + 1> sigmas, Eigen::Vector<float, 2 * N + 1> wm) {
    Eigen::Vector<float, N> x;

    float yawsum_sin = 0;
    float yawsum_cos = 0;
    float pitchsum_sin = 0;
    float pitchsum_cos = 0;
    float rollsum_sin = 0;
    float rollsum_cos = 0;

    for (int i = 0; i < (2 * N + 1); i++) {
        // assume x,y,theta...... rewrite for glider :sob:
        x(0) += sigmas(0, i) * wm(i);
        x(1) += sigmas(1, i) * wm(i);
        x(2) += sigmas(2, i) * wm(i);
        x(3) += sigmas(3, i) * wm(i);
        x(4) += sigmas(4, i) * wm(i);
        x(5) += sigmas(5, i) * wm(i);
        yawsum_sin += std::sin(sigmas(2, i)) * wm(i);
        yawsum_cos += std::cos(sigmas(2, i)) * wm(i);
        pitchsum_sin += std::sin(sigmas(2, i)) * wm(i);
        pitchsum_cos += std::cos(sigmas(2, i)) * wm(i);
        rollsum_sin += std::sin(sigmas(2, i)) * wm(i);
        rollsum_cos += std::cos(sigmas(2, i)) * wm(i);
    }

    x(6) = std::atan2(yawsum_sin, yawsum_cos);
    x(7) = std::atan2(pitchsum_sin, pitchsum_cos);
    x(8) = std::atan2(rollsum_sin, rollsum_cos);
    return x;
}

Eigen::Vector<float, ROWS> sr_ukf::measurement_mean(Eigen::Matrix<float, ROWS, 2 * N + 1> sigmas, Eigen::Vector<float, 2 * N + 1> wm) {
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

Eigen::Vector<float, N> sr_ukf::state_residual(Eigen::Vector<float, N> a, Eigen::Vector<float, N> b) {
    Eigen::Vector<float, N> out = a - b;
    out(6) = normalize_angle180(out(6));
    out(7) = normalize_angle180(out(7));
    out(8) = normalize_angle180(out(8));

    return out;
}

Eigen::Vector<float, N> sr_ukf::state_add(Eigen::Vector<float, N> a, Eigen::Vector<float, N> b) {
    Eigen::Vector<float, N> out = a + b;
    out(6) = normalize_angle180(out(6));
    out(7) = normalize_angle180(out(7));
    out(8) = normalize_angle180(out(8));

    return out;
}

Eigen::Vector<float, ROWS> sr_ukf::measurement_residual(Eigen::Vector<float, ROWS> a, Eigen::Vector<float, ROWS> b) {
    Eigen::Vector<float, ROWS> out = a - b;
    out(2) = normalize_angle180(out(2));

    return out;
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

