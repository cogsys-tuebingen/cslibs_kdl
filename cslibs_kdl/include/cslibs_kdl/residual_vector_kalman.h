#ifndef CS_RESIDUAL_VECTOR_KALMAN_H
#define CS_RESIDUAL_VECTOR_KALMAN_H

#include <vector>
#include <Eigen/Core>
class ResidualVectorKalman
{
public:
    ResidualVectorKalman();


    void initialize(std::size_t n, const Eigen::VectorXd& state);
    void initialize(const std::vector<double> & state);
    void setDimensions(std::size_t n);
    void setCovarianceProcess(const Eigen::MatrixXd& Q);
    void setCovarianceMeasurment(const Eigen::MatrixXd& R);

    Eigen::VectorXd update(Eigen::VectorXd& measurement);
    void update(const std::vector<double> &measurement, std::vector<double> &result);

    Eigen::MatrixXd getCovariance() const;

private:
    std::size_t n_;
    Eigen::VectorXd state_;
    Eigen::MatrixXd cov_state_;
    Eigen::MatrixXd cov_measurment_;
    Eigen::MatrixXd cov_;
    Eigen::MatrixXd eye_;

};

#endif // CS_RESIDUAL_VECTOR_KALMAN_H
