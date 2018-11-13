#include <cslibs_kdl/residual_vector_kalman.h>
#include <cslibs_kdl/kdl_conversion.h>
#include <Eigen/Dense>
ResidualVectorKalman::ResidualVectorKalman():
    n_(0)
{

}

void ResidualVectorKalman::setDimensions(std::size_t n)
{
    n_ = n;
    eye_ = Eigen::MatrixXd::Identity(n,n);
}
void ResidualVectorKalman::initialize(std::size_t n, const Eigen::VectorXd& state)
{
    state_ = state;
    n_ = n;
    eye_ = Eigen::MatrixXd::Identity(n_,n_);
    cov_ = Eigen::MatrixXd::Identity(n_,n_);
}

void ResidualVectorKalman::initialize(const std::vector<double> &state)
{
    cslibs_kdl::convert(state, state_);
    n_ = state.size();
    eye_ = Eigen::MatrixXd::Identity(n_,n_);
    cov_ = Eigen::MatrixXd::Identity(n_,n_);
}

void ResidualVectorKalman::setCovarianceProcess(const Eigen::MatrixXd& Q)
{
    cov_state_ = Q;
}

void ResidualVectorKalman::setCovarianceMeasurment(const Eigen::MatrixXd &R)
{
    cov_measurment_ = R;
}

Eigen::VectorXd ResidualVectorKalman::update(Eigen::VectorXd& measurement)
{

    Eigen::MatrixXd cov_pri = cov_ + cov_measurment_;

    Eigen::MatrixXd K = cov_pri * (cov_pri + cov_state_).inverse();
    state_ += K *(measurement - state_);
    cov_ = (eye_ - K)*cov_pri;

    return state_;

}

void ResidualVectorKalman::update(const std::vector<double>& measurement, std::vector<double>& result)
{
    Eigen::VectorXd meas;
    cslibs_kdl::convert(measurement, meas);
    Eigen::VectorXd res = update(meas);
    cslibs_kdl::convert(res, result);
}

Eigen::MatrixXd ResidualVectorKalman::getCovariance() const
{
    return cov_;
}
