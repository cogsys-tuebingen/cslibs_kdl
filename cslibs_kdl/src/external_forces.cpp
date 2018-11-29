#include <cslibs_kdl/external_forces.h>
#include <cslibs_kdl/kdl_conversion.h>

using namespace cslibs_kdl;

ExternalForcesSerialChain::ExternalForcesSerialChain() :
    set_model_(false),
    init_sensor_mat_(false),
    n_joints_(0),
    end_eff_frame_("")//,
{

}

bool ExternalForcesSerialChain::initialize()
{
    if(!set_model_){
        return false;
    }

    std::size_t nj_sq = n_joints_ * n_joints_;

    if(!init_sensor_mat_){
        sensor_mat_.setZero(n_joints_, nj_sq);

        std::size_t row = 0;

        for(auto link : link_names_){
            KDL::Vector rot_axis;
            model_.getRotationAxis(link, rot_axis);
//            Eigen::Matrix<double,1,6> si;
//            si << rot_axis(0),rot_axis(1),rot_axis(2),0,0,0;
//            sensor_mat_.block(row,6*row,1,6) = si;
//            sensor_mat_.block(row, 6*row,1,6) << rot_axis(0),rot_axis(1),rot_axis(2),0,0,0;
            sensor_mat_(row, 6*row)     = rot_axis(0);
            sensor_mat_(row, 6*row + 1) = rot_axis(1);
            sensor_mat_(row, 6*row + 2) = rot_axis(2);
            ++row;
        }
    }
    init_sensor_mat_ = true;

    return true;
}

void ExternalForcesSerialChain::setModel(const std::string &robot_model,
                              const std::string &chain_root,
                              const std::string &chain_tip,
                              const std::string &finger_1_tip,
                              const std::string &finger_2_tip,
                              const std::string &finger_3_tip)
{
    robot_model_ = robot_model;
    chain_root_ = chain_root;
    chain_tip_ = chain_tip;
    finger_1_tip_ = finger_1_tip;
    finger_2_tip_ = finger_2_tip;
    finger_3_tip_ = finger_3_tip;
    model_= cslibs_kdl::KinematicModel(robot_model, chain_root, chain_tip);
    model_f1_= cslibs_kdl::KinematicModel(robot_model, chain_tip, finger_1_tip);
    model_f2_= cslibs_kdl::KinematicModel(robot_model, chain_tip, finger_2_tip);
    model_f3_= cslibs_kdl::KinematicModel(robot_model, chain_tip, finger_3_tip);
    link_names_ = model_.getLinkNames();
    n_joints_ = model_.getNrOfJoints();
    set_model_ = true;
    init_sensor_mat_ = false;
}
void ExternalForcesSerialChain::getGeometricJacobianTransposed(const std::vector<double>& pos,
                                    std::size_t n_joints,
                                    Eigen::MatrixXd& result ) const
{
    if(!init_sensor_mat_){
        return;
    }
    std::size_t max = n_joints;
    if(n_joints > n_joints_){
        max = n_joints_;
    }


    result.setZero(max, 6);
    std::string frame_end = link_names_[max -1];
    for(std::size_t row = 0; row < max; ++row){
        std::string link = link_names_[row];
        KDL::Frame T0i;
        model_.getFKPose(pos, T0i, link);
//        KDL::Vector rot_axis = rot_axis_[row];
        KDL::Frame T0j;
        model_.getFKPose(pos, T0j, frame_end);
        KDL::Frame Tij = T0i.Inverse() * T0j;
        Eigen::Matrix<double, 6, 6> T = cslibs_kdl::convert2EigenWrenchTransform(Tij);
//        Eigen::Matrix<double,1,6> si = sensor_mat_.block(row,6*row,1,6).eval();
        Eigen::Matrix<double,1,6>  column = sensor_mat_.block(row,6*row,1,6).eval() * T;
        result.block(row,0,1,6) =  column;
    }
}

void ExternalForcesSerialChain::getGeometricJacobianTransposed(const std::vector<double> &pos, std::string segment, Eigen::MatrixXd &result) const
{

    std::string frame = segment;
    if(segment.find("finger") != std::string::npos ){
        frame = chain_tip_;
    }
    int index = model_.getKDLSegmentIndex(frame);
    if(index < 0){
        throw std::runtime_error("Cannot find link id for " + segment);
    }

    getGeometricJacobianTransposed(pos,index +1, result);
}

void ExternalForcesSerialChain::getGeometricJacobianTransposed(const cslibs_kdl_data::JointStateData &state, std::size_t n_joints, Eigen::MatrixXd &result) const
{
    getGeometricJacobianTransposed(state.position, n_joints, result);
}

void ExternalForcesSerialChain::getWrenchProjetion(const cslibs_kdl_data::JointStateData& state, Eigen::MatrixXd& result ) const
{
    std::size_t nj_sq = n_joints_ * n_joints_;
    result.setZero(nj_sq, nj_sq);

    std::size_t row = 0;

    for(const::std::string& link : link_names_){
        KDL::Frame T0i;
        model_.getFKPose(state.position, T0i, link);
        for(std::size_t col = row; col < n_joints_; ++col){
            KDL::Frame T0j;
            model_.getFKPose(state.position, T0j, link_names_[col]);
            KDL::Frame Tij = T0i.Inverse() * T0j;
            result.block<6,6>(6*row,6*col) = cslibs_kdl::convert2EigenWrenchTransform(Tij);
        }
        ++row;
    }
}

void ExternalForcesSerialChain::getWrenchProjetion(const cslibs_kdl_data::JointStateData& state,
                                      std::size_t up_to_joint,
                                      Eigen::MatrixXd& result ) const
{
    std::size_t nj_sq = n_joints_ * n_joints_;
    result.setZero(nj_sq, nj_sq);

    for(std::size_t row = 0; row <= up_to_joint; ++row){
        std::string link = link_names_[row];
        KDL::Frame T0i;
        model_.getFKPose(state.position, T0i, link);
        for(std::size_t col = row; col <= up_to_joint; ++col){
            KDL::Frame T0j;
            model_.getFKPose(state.position, T0j, link_names_[col]);
            KDL::Frame Tij = T0i.Inverse() * T0j;
            result.block<6,6>(6*row,6*col) = cslibs_kdl::convert2EigenWrenchTransform(Tij);
        }
        ++row;
    }
}

Eigen::MatrixXd ExternalForcesSerialChain::getJacobian(const std::vector<double> &pos, int id)
{

    return model_.getJacobian(pos, id).data;
}

Eigen::MatrixXd ExternalForcesSerialChain::getJacobian(const cslibs_kdl_data::JointStateData &state, int id)
{
    return model_.getJacobian(state.position, id).data;
}

bool ExternalForcesSerialChain::getJacobian(const cslibs_kdl_data::JointStateData &state, const std::string &frame, Eigen::MatrixXd& jacobian)
{
    KDL::Chain chain;
    bool succ = model_.getTree().getChain(chain_root_, frame, chain);
    if(succ){
        KDL::ChainJntToJacSolver jac_solver(chain);
        if(state.position.size() > n_joints_ ){
            throw std::logic_error("Dimension mismatch. More joint values expected");
        }
        KDL::Jacobian jac(chain.getNrOfJoints());

        KDL::JntArray qvals;
        cslibs_kdl::convert(state.position, qvals, state.position.size() - chain.getNrOfJoints());

        jac_solver.JntToJac(qvals, jac);
        jacobian = jac.data;
    }
    return succ;
}

Eigen::VectorXd ExternalForcesSerialChain::getExternalTorques(const std::vector<double>& pos,
                                   std::string frame_id,
                                   const KDL::Wrench &w) const
{
    Eigen::MatrixXd J;
    KDL::Wrench wt = w;

    std::string frame = frame_id;
    if(frame_id.find("finger") != std::string::npos ){
        frame = chain_tip_;
        KDL::Frame hTf = getFKPose(pos, chain_tip_);
        wt = hTf * wt;
    }

    Eigen::VectorXd F = cslibs_kdl::convert2Eigen(wt);

    int index = model_.getKDLSegmentIndex(frame);
    if(index < 0){
        throw std::runtime_error("Cannot find link id for " + frame_id);
    }

    getGeometricJacobianTransposed(pos,index +1, J);

    //        geometry_msgs::WrenchStamped msg;
    //        msg.wrench.force.x = n_i.force.x();
    //        msg.wrench.force.y = n_i.force.y();
    //        msg.wrench.force.z = n_i.force.z();
    //        msg.wrench.torque.x = n_i.torque.x();
    //        msg.wrench.torque.y = n_i.torque.y();
    //        msg.wrench.torque.z = n_i.torque.z();
    //        msg.header.frame_id = link_names_[id];
    //        msg.header.stamp = ros::Time::now();
    //        pubTorques.publish(msg);
    //        ros::spinOnce();

//    res = sensor_mat_ * J * F;
    Eigen::VectorXd res = J * F;
    return res;
}

Eigen::VectorXd ExternalForcesSerialChain::getExternalTorquesKDL(const std::vector<double>& pos, std::string frame_id, KDL::Wrench &w_local) const
{
    std::string frame = frame_id;
    KDL::Wrench wchain = w_local;
    // if local frame is a finger frame project wrench to finger frame
    if(frame_id.find("finger") != std::string::npos ){
        frame = chain_tip_;
        KDL::Frame hTf = getFKPose(pos, chain_tip_);
        wchain = hTf * wchain;
    }

    int index = model_.getKDLSegmentIndex(frame);
    if(index < 0){
        throw std::runtime_error("Cannot find link id for " + frame_id);
    }

    // rotate wrench in to base frame coordinates
    KDL::Frame baseTframe = getFKPose(pos, frame_id);
    KDL::Wrench w_in(baseTframe.M * wchain.force, baseTframe.M * wchain.torque);

    KDL::Jacobian jac = model_.getJacobian(pos, index +1);
    Eigen::MatrixXd jac_t = jac.data.transpose();
    Eigen::Matrix<double, 6, 1> ft;
    ft << w_in.force(0) ,  w_in.force(1) ,w_in.force(2),
            w_in.torque(0) , w_in.torque(1) , w_in.torque(2);
    Eigen::VectorXd tau = jac_t * ft;
    return tau;
}

Eigen::VectorXd ExternalForcesSerialChain::getExternalTorques(const cslibs_kdl_data::JointStateData &state,
                                   std::string frame_id,
                                   const KDL::Wrench& w) const
{

    return getExternalTorques(state.position, frame_id, w);
}

Eigen::VectorXd ExternalForcesSerialChain::getExternalTorquesKDL(const cslibs_kdl_data::JointStateData &state, std::string frame_id, KDL::Wrench& w_local) const
{
    return getExternalTorquesKDL(state.position, frame_id, w_local);
}

KDL::Frame ExternalForcesSerialChain::getFKPose(const std::vector<double>& position, const std::string& link) const
{
    KDL::Frame res;
    int ec = 0;
    auto getQvec = [](const std::vector<double>& pos, const std::size_t& n_joints, const std::size_t offset){
         std::vector<double> q(2,0);
        if(pos.size() > n_joints + offset){
            q[0] = pos[n_joints + offset];
        }
        return q;
    };

    if(link.find("finger") == std::string::npos){
        ec = model_.getFKPose(position, res, link);
    }
    else if(link.find("1") != std::string::npos ){
        KDL::Frame hand;
        KDL::Frame f1;
        ec = model_.getFKPose(position, hand, model_.getTipLink());
        std::vector<double> q = getQvec(position, n_joints_, 0);
        ec *= model_f1_.getFKPose(q, f1, link);
        res = hand * f1;
    }
    else if(link.find("2") != std::string::npos ){
        KDL::Frame hand;
        KDL::Frame f2;
        ec = model_.getFKPose(position, hand, model_.getTipLink());
        std::vector<double> q = getQvec(position, n_joints_, 1);
        ec *= model_f2_.getFKPose(q, f2, link);
        res = hand * f2;
    }
    else if(link.find("3") != std::string::npos ){
        KDL::Frame hand;
        KDL::Frame f3;
        ec = model_.getFKPose(position, hand, model_.getTipLink());
        std::vector<double> q = getQvec(position, n_joints_, 3);
        ec *= model_f3_.getFKPose(q, f3, link);
        res = hand * f3;
    }
    if(ec < 0){
        throw std::runtime_error("can not compute forwad kinematics.");
    }
    return res;
}

KDL::Frame ExternalForcesSerialChain::getFKPose(const std::vector<double> &pos, const std::string& parent, const std::string& link) const
{
    // FK returns pose in base frame coordinates!
    KDL::Frame b_T_p = getFKPose(pos, parent);
    KDL::Frame b_T_l = getFKPose(pos, link);
    KDL::Frame p_T_l = b_T_p.Inverse() * b_T_l;
    return p_T_l;
}

KDL::Frame ExternalForcesSerialChain::getFKPose(const cslibs_kdl_data::JointStateData &state, const std::string& link) const
{
    return getFKPose(state.position, link);
}
std::vector<std::string> ExternalForcesSerialChain::getAllLinkNames() const
{
    std::vector<std::string> res = model_.getLinkNames();
    std::vector<std::string> f1 = model_f1_.getLinkNames();
    std::vector<std::string> f2 = model_f2_.getLinkNames();
    std::vector<std::string> f3 = model_f3_.getLinkNames();
    for(auto s : f1){
        res.emplace_back(s);
    }
    for(auto s : f2){
        res.emplace_back(s);
    }
    for(auto s : f3){
        res.emplace_back(s);
    }
    return res;
}

KDL::Wrench ExternalForcesSerialChain::createWrench(const KDL::Vector& position, const KDL::Vector& direction_vector, const KDL::Vector& force)
{
    KDL::Vector x(1,0,0);
    KDL::Vector axis = x * direction_vector;
    KDL::Wrench w(force, KDL::Vector::Zero());
    double alpha = std::acos(dot(x, direction_vector));
    KDL::Frame T(KDL::Rotation::Rot(axis, alpha), position);
    w = T * w;
    return w;
}

KDL::Wrench ExternalForcesSerialChain::createWrench(const KDL::Vector& position, const KDL::Vector& direction_vector, const double theta, const double phi)
{
    KDL::Vector z(0,0,1);
    KDL::Vector axis = z * direction_vector;
    KDL::Vector force(std::sin(theta)*std::cos(phi),
                      std::sin(theta)*std::sin(phi),
                      std::cos(theta));
    KDL::Wrench w(-force, KDL::Vector::Zero());
    double alpha = std::acos(dot(z, direction_vector));
    KDL::Frame T(KDL::Rotation::Rot(axis, alpha), position);
    w = T * w;
    return w;
}
