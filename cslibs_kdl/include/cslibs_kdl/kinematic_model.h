#ifndef CS_KINEMATICMODEL_H
#define CS_KINEMATICMODEL_H
//System
#include <string>
#include <vector>
#include <memory>
#include <random>
#include <cslibs_kdl_data/suppress_warnings_start.h>
//ROS
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <tf/tf.h>
// trac_ik
#include <trac_ik/trac_ik.hpp>
//Orocos KDL
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/solveri.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <cslibs_kdl_data/suppress_warnings_end.h>
namespace cslibs_kdl {


class KinematicModel
{
public:
    KinematicModel();
    KinematicModel(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);

    virtual ~KinematicModel();

    virtual bool setTreeParam(const std::string& robot_model);
    virtual bool setTreeFile(const std::string &robot_model);
    void setRootAndTip(const std::string& chain_root, const std::string& chain_tip);
    void setGravity(double x, double y, double z);
    void getGravity(double& gx, double& gy, double& gz);

    //Solver Calls

    /**
     * @brief getFKPose Given the joint variables q_in the world space coordinates are calculated. KDL wrapper
     * @param q_in  input: joint variables
     * @param out   output: pose (position and orientation)
     * @param link  input: the link for which the pose should be calculated.
     * @return
     */
    int getFKPose(const std::vector<double>& q_in, tf::Pose& out, const std::string link) const;

    /**
     * @brief getFKPose Given the joint variables q_in the world space coordinates are calculated. KDL wrapper
     * @param q_in  input: joint variables
     * @param out   output: pose (position and orientation)
     * @param link  input: the link for which the pose should be calculated.
     * @return
     */
    int getFKPose(const std::vector<double>& q_in, KDL::Frame& out, const std::string link) const;

    /**
     * @brief getIKSolution given a end effector pose a corresponding joint configuration is search. trac_ik wrapper
     * @param pose      input: the pose (position, orientation) of the end-effector
     * @param result    output: the joint variables.
     * @param seed      optional input: seed for joint value search. if not given a random seed is choosen.
     * @return
     */
    int getIKSolution(const tf::Pose &pose, std::vector<double> &result, const std::vector<double>& seed = std::vector<double>());

    void changeKineticParams(const std::string& link, const Eigen::Vector3d& trans, const Eigen::Matrix3d& rotation);

    void getRandomConfig(std::vector<double>& config);
    /**
     * @brief getKDLSegmentIndex gets the KDL segment index for a given segment/ link name
     * @param name the name of the segment/ link
     * @return returns index of segment -1 if name == root_ ; -2 if link name not found.
     */
    int getKDLSegmentIndex(const std::string &name ) const;

    void getRandomConfig(Eigen::VectorXd& config);

    void useUrdfDynamicParams();
    /**
     * @brief getKDLSegmentIndexFK gets the index of a link as needed for forward kinematics. root = 0, ...
     * @param name name of the segment/link.
     * @return the index of the segment/link.
     */
    int getKDLSegmentIndexFK(const std::string &name) const;
    int getNrOfJoints() const {return chain_.getNrOfJoints();}
    int getNrOfSegments() const {return chain_.getNrOfJoints();}

    std::vector<std::string> getLinkNames() const;
    std::string getRootLink() const {return root_;}
    std::string getTipLink() const {return tip_;}

    double getUpperJointLimit(const std::size_t id) const;
    double getLowerJointLimit(const std::size_t id) const;


    Eigen::Vector3d getLinkFixedTranslation(const std::string &link) const;
    Eigen::Matrix3d getLinkFixedRotation(const std::string &link) const;

    /**
     * @brief getRotationAxis gets the rotation axis of the link in the link frame
     * @param link the name of the link
     * @param rot_axis the rotation axis normaly z-axis (DH connvention)
     */
    void getRotationAxis(const std::string &link, KDL::Vector &rot_axis) const;
    void getRotationAxis(const std::string &link, Eigen::Vector3d& rot_axis) const;
    KDL::Twist getJointAxisProjection(const std::string& link) const;
    std::vector<KDL::Twist> getJointAxisProjections() const;

    /**
     * @brief getJacobian computes the geometric Jacobian of input frame
     * @param q joint angles
     * @param seg_id identifier of segment default is end effector
     * @return
     */
    KDL::Jacobian getJacobian(const std::vector<double>& q, int seg_id = -1) const;
    int getJointVelocities(const std::vector<double> &q, const KDL::Twist &v_in, std::vector<double>& v_out);
    inline const KDL::Tree& getTree() const {return tree_;}

protected:
    virtual bool initialize();

protected:
    std::string urdf_param_;
    std::string root_;
    std::string tip_;
    urdf::Model robot_model_;
    KDL::Tree tree_;
    KDL::Chain chain_;
    KDL::Chain chainFile_;
    KDL::Vector gravity_;

    std::shared_ptr<KDL::ChainFkSolverPos_recursive> solverFk_;
    std::shared_ptr<TRAC_IK::TRAC_IK> solverIK_;
    std::shared_ptr<KDL::ChainJntToJacSolver> solverJac_;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> solverIKVel_;
    std::vector<std::uniform_real_distribution<double> > jointDist_;
    std::default_random_engine randEng_;
    KDL::JntArray lowerLimits_;
    KDL::JntArray upperLimits_;




};
}

#endif // CS_KINEMATICMODEL_H
