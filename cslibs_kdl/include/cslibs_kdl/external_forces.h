#ifndef EXTERNAL_FORCES_H
#define EXTERNAL_FORCES_H

#include <cslibs_kdl/dynamic_model.h>
#include <cslibs_kdl_data/joint_state_data.h>
#include <Eigen/Core>

namespace cslibs_kdl {
class ExternalForcesSerialChain
{
public:
    using Ptr = std::shared_ptr<ExternalForcesSerialChain>;
    using ConstPtr = std::shared_ptr<const ExternalForcesSerialChain>;
public:
    ExternalForcesSerialChain();

    bool initialize();

    void setModel(const std::string &robot_model,
                  const std::string &chain_root,
                  const std::string &chain_tip,
                  const std::string &finger_1_tip,
                  const std::string &finger_2_tip,
                  const std::string &finger_3_tip);

    void getGeometricJacobianTransposed(const cslibs_kdl_data::JointStateData& state,
                                        std::size_t up_to_joint,
                                        Eigen::MatrixXd& result ) const;

    /**
     * @brief getForceJacobian calculates the "force" Jacobian relating a vector of external wrenches F in R^(n*6)
     *  for each joint to the acting joint torques
     * @param state joint state containing the required joint positions
     * @param result a matrix of dimension (n*6) x (n*6) n = number of joints.
     */
    void getWrenchProjetion(const cslibs_kdl_data::JointStateData& state,
                            Eigen::MatrixXd& result) const;

    /**
     * @brief getForceJacobian calculates the "force" Jacobian relating a vector of external wrenches F in R^(n*6)
     *  for each joint to the acting joint torques
     * @param state joint state containing the required joint positions
     * @param up_to_joint last required joint
     * @param result a matrix of dimension (n*6) x (n*6) n = number of joints.
     */
    void getWrenchProjetion(const cslibs_kdl_data::JointStateData& state,
                            std::size_t up_to_joint,
                            Eigen::MatrixXd& result ) const;
    KDL::Frame getFKPose(const cslibs_kdl_data::JointStateData &state, const std::string& link) const;

    Eigen::VectorXd getExternalTorques(const cslibs_kdl_data::JointStateData &state,
                                       std::string frame_id,
                                       const KDL::Wrench &w) const;
    Eigen::VectorXd getExternalTorquesKDL(const cslibs_kdl_data::JointStateData& state,
                                          std::string frame_id, KDL::Wrench &w_local) const ;

    Eigen::MatrixXd getJacobian(const cslibs_kdl_data::JointStateData &state);
    bool getJacobian(const cslibs_kdl_data::JointStateData& state, const std::string& frame, Eigen::MatrixXd &jacobian);

    inline std::size_t getNrJoints() const {return n_joints_;}
    inline Eigen::MatrixXd getJointAxisProjection() const {return sensor_mat_;}
    inline std::string getFrameId(std::size_t id) const {return link_names_.at(id);}

protected:
    bool set_model_;
    bool init_sensor_mat_;
    KinematicModel model_;
    KinematicModel model_f1_;
    KinematicModel model_f2_;
    KinematicModel model_f3_;
    std::size_t n_joints_;
    Eigen::MatrixXd sensor_mat_;
    std::vector<std::string> link_names_;
    //    std::vector<double> link_length_;
    //    std::vector<double> link_elongation_;
    //    std::vector<KDL::Frame> link_trans_direction_;
    //    double arm_length_;
    //    double l_last_link_;
    std::string end_eff_frame_;
    tf::Transform end_eff_trans_;
    std::string robot_model_;
    std::string chain_root_;
    std::string chain_tip_;
    std::string finger_1_tip_;
    std::string finger_2_tip_;
    std::string finger_3_tip_;

};
}
#endif // EXTERNAL_FORCES_H