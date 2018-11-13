#include <cslibs_kdl/residual_vector.h>
using namespace cslibs_kdl;

ResidualVector::ResidualVector()
    :initial_(true), model_()
{

}

ResidualVector::ResidualVector(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip)
    : initial_(true),model_(robot_model, chain_root, chain_tip)
{

}


void ResidualVector::setGains(std::vector<double> &gains)
{
    gains_.resize(gains.size(),gains.size());
    gains_.setZero(gains.size(),gains.size());
    std::size_t i = 0;
    for(auto value : gains){
        gains_(i,i) = value;
        ++i;
    }
}

void ResidualVector::setGravity(double x, double y, double z)
{
    model_.setGravity(x,y,z);
}

int ResidualVector::getNrOfJoints()const
{
    int val = model_.getNrOfJoints();
    return val;
}

void ResidualVector::setTree(const std::string &robot_model)
{
    model_.setTreeParam(robot_model);
}

void ResidualVector::setRootAndTip(const std::string &chain_root, const std::string &chain_tip)
{
    model_.setRootAndTip(chain_root, chain_tip);
}

void ResidualVector::changeDynamicParams(const std::string &link, const double mass, const Eigen::Vector3d &com, const Eigen::Matrix3d &inertia)
{
    model_.changeDynamicParams(link, mass, com, inertia);
}

std::vector<double> ResidualVector::getGravityTorque() const
{
    std::size_t njoints = model_.getNrOfJoints();
    std::vector<double> res(njoints);
    for(std::size_t i = 0; i < njoints; ++i){
        res[i] = gravity_torque_(i);
    }
    return res;

}

void ResidualVector::getResidualVector(std::vector<ResidualData> &sequence, std::vector<Eigen::VectorXd> &residual_vec) const
{
    // initialization
    std::size_t n_joints =  model_.getNrOfJoints();
    Eigen::VectorXd r0(n_joints);
    r0.setZero(n_joints);

    residual_vec.resize(sequence.size());

    auto it_residual_vec = residual_vec.begin();
    *it_residual_vec = r0;

    Eigen::VectorXd last_integral(n_joints);
    last_integral.setZero(n_joints);

    for(auto data : sequence){

        model_.setGravity(data.gx, data.gy, data.gz);
        Eigen::MatrixXd C;
        model_.getMatrixC(data.joint_positions,data.joint_velocities, C);

        Eigen::VectorXd omega;
        Eigen::VectorXd tau;
        vector2EigenVector(data.joint_velocities, omega);
        vector2EigenVector(data.torques, tau);

        Eigen::MatrixXd H;
        Eigen::VectorXd G;
        model_.getChainDynInertiaAndGravity(data.joint_positions, H, G);
        Eigen::VectorXd m = H * omega;
        Eigen::VectorXd to_integrate = tau + Eigen::Transpose<Eigen::MatrixXd>(C) * omega - G + *it_residual_vec;

        Eigen::VectorXd integral = integration_step(data.dt, last_integral, to_integrate);
        last_integral = integral;

        *it_residual_vec = gains_ * (m - integral);
        ++it_residual_vec;
    }


}

void ResidualVector::getResidualVector(std::vector<ResidualData> &sequence, std::vector<std::vector<double> > &residual_vec) const
{
    std::vector<Eigen::VectorXd> res;
    getResidualVector(sequence, res);
    residual_vec.resize(sequence.size());
    auto it_res_vec = residual_vec.begin();
    for(auto data : res){
        eigenVector2vector(data, *it_res_vec);
    }
}

void ResidualVector::getResidualVector(const ResidualData &data,
                                            const Eigen::VectorXd& last_residual,
                                            const Eigen::VectorXd& last_integral,
                                            Eigen::VectorXd& new_integral,
                                            Eigen::VectorXd& new_residual)
{

    if(/*data.dt < 1e-7 || */data.dt > 0.1){
        new_integral = last_integral;
        new_residual = last_residual;
        ROS_WARN_STREAM("Time difference very small or very big: " << data.dt);
        return;
    }
//    ROS_INFO_STREAM("getResidualVector dt: " << data.dt);
    Eigen::MatrixXd C;
    model_.getMatrixC(data.joint_positions,data.joint_velocities, C);

    Eigen::VectorXd omega;
    Eigen::VectorXd tau;
    vector2EigenVector(data.joint_velocities, omega);
    vector2EigenVector(data.torques, tau);

    Eigen::MatrixXd H;
//    Eigen::VectorXd G;
    model_.setGravity(data.gx, data.gy, data.gz);
//    model_.getChainDynInertiaAndGravity(data.joint_positions, H, G);
//    Eigen::VectorXd m = H * omega;
//    Eigen::VectorXd to_integrate = tau + Eigen::Transpose<Eigen::MatrixXd>(C) * omega - G + last_residual;
    model_.getChainDynInertiaAndGravity(data.joint_positions, H, gravity_torque_);
    Eigen::VectorXd m = H * omega;
    Eigen::VectorXd to_integrate = tau + Eigen::Transpose<Eigen::MatrixXd>(C) * omega - gravity_torque_ + last_residual; // changed sign at 15.17.2017

    new_integral = integration_step(data.dt, last_integral, to_integrate);

    new_residual = gains_ * (m - new_integral);


}
//TODO TEST THIS
void ResidualVector::getResidualVector(const cslibs_kdl_data::JointStateDataStamped &data_s,
                                            const Eigen::VectorXd& last_residual,
                                            const Eigen::VectorXd& last_integral,
                                            Eigen::VectorXd& new_integral,
                                            Eigen::VectorXd& new_residual)
{
    ROS_WARN("untested method");
    bool init = initial_;
    if(initial_){
        last_stamp_ = data_s.stamp();
        initial_ = false;
    }
    double dt = data_s.stamp().substractionResultInSeconds(last_stamp_);
    last_stamp_ = data_s.stamp();

    if(dt < 1e-7 || dt > 0.1){
        new_integral = last_integral;
        new_residual = last_residual;
        if(!init){
            ROS_WARN_STREAM("Time difference very small or very big: " << dt);
        }

        return;
    }
    const cslibs_kdl_data::JointStateData& data = data_s.data;
    Eigen::MatrixXd C;
    model_.getMatrixC(data.position, data.velocity, C);

    Eigen::VectorXd omega;
    Eigen::VectorXd tau;
    vector2EigenVector(data.velocity, omega);
    vector2EigenVector(data.torque, tau);

    Eigen::MatrixXd H;
    model_.setGravity(data.gravity(0), data.gravity(1), data.gravity(2));
    model_.getChainDynInertiaAndGravity(data.position, H, gravity_torque_);
    Eigen::VectorXd m = H * omega;
    Eigen::VectorXd to_integrate = tau + Eigen::Transpose<Eigen::MatrixXd>(C) * omega - gravity_torque_ + last_residual;

    new_integral = integration_step(dt, last_integral, to_integrate);
    new_residual = gains_ * (m - new_integral);

}



Eigen::VectorXd ResidualVector::integration_step(const double dt, const Eigen::VectorXd &last_integral, const Eigen::VectorXd &next_integrant) const
{
    Eigen::VectorXd result = last_integral + dt * next_integrant;
    return result;
}

int ResidualVector::getAcceleration(const std::vector<double> &q,
                                         const std::vector<double> &q_Dot,
                                         const std::vector<double> &q_DotDot,
                                         std::vector<std::string> &links,
                                         std::vector<KDL::Twist> &spatial_acc)
{
    return model_.getAcceleration(q, q_Dot, q_DotDot, links, spatial_acc);
}

std::vector<std::string> ResidualVector::getLinkNames() const
{
    return model_.getLinkNames();
}

void ResidualVector::vector2EigenVector(const std::vector<double> &vec, Eigen::VectorXd &res)
{
    res.resize(vec.size());
    std::size_t i = 0;
    for(auto data : vec)
    {
        res(i) = data;
        ++i;
    }
}

void ResidualVector::eigenVector2vector(const Eigen::VectorXd &vec, std::vector<double> &res)
{
    res.resize(vec.rows());
    for(std::size_t i = 0; i < vec.rows(); ++i){
        res[i] = vec(i);
    }
}
