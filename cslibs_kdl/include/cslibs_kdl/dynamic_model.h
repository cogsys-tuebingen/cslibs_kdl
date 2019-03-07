#ifndef CS_DynamicModel_H
#define CS_DynamicModel_H
//System
#include <string>
#include <vector>
#include <memory>
//ROS
#include <cslibs_kdl_data/suppress_warnings_start.h>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <tf/tf.h>
// trac_ik
#include <trac_ik/trac_ik.hpp>
//Orocos KDL
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/solveri.hpp>
#include <kdl/chaindynparam.hpp>
#include <cslibs_kdl_data/suppress_warnings_end.h>
#include <cslibs_kdl/kinematic_model.h>
#include <cslibs_kdl_data/wrench.h>
namespace cslibs_kdl {

/**
 * @brief The DynamicModel class
 *
 * Inverse Dynamics: torques  = H*qdotdot + C(q,qdot,f) + G(q)
 *                   qdotdot:  joint acceleration
 *                   qdot:     joint velocity
 *                   q:        joint position
 */
class DynamicModel : public KinematicModel
{
public:
    DynamicModel();
    DynamicModel(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);


    bool setTreeParam(const std::string& robot_model) override;
//    bool setTreeFile(const std::string &robot_model) override;
    /**
     * @brief setGravity set gravity
     * @param x x component (default 0)
     * @param y y component (default 0)
     * @param z z component (default -9.81)
     */
    void setGravity(double x, double y, double z);
    /**
     * @brief getGravity gets gravity
     * @param gx x component (default 0)
     * @param gy y component (default 0)
     * @param gz z component (default -9.81)
     */
    void getGravity(double& gx, double& gy, double& gz);

    //Solver Calls
    /**
     * @brief getTorques Inverse dynamics solver call. Wrapper for KDL Recursive Newton Euler Algorithem cf. orocos project and "Rigid Body Dynamics Algorithms" of Roy Featherstone, 2008 (ISBN:978-0-387-74314-1) See page 96 for the pseudo-code.
     * @param q         input: the joint variables, positions
     * @param q_Dot     input: joint velocities
     * @param q_DotDot  input: joint accelerations
     * @param torques   output: joint torques
     * @param wrenches_ext optional input: external forces (not gravity)
     * @return 0 if successful
     */
    virtual int getTorques(const std::vector<double>& q, const std::vector<double>& q_Dot, const std::vector<double>& q_DotDot,
                           std::vector<double>& torques, const std::vector<cslibs_kdl_data::Wrench>& wrenches_ext = std::vector<cslibs_kdl_data::Wrench>());

    virtual int getTorques(const std::vector<double>& q, const std::vector<double>& q_Dot, const std::vector<double>& q_DotDot,
                           std::vector<double>& torques, const std::vector<KDL::Wrench>& wrenches_ext);

    /**
     * @brief getJointAcceleration Forward Dynamic solver call. Inertia Matrix method
     * @param q             input: the joint variables, positions
     * @param q_Dot         input: joint velocities
     * @param torques       input: joint torques
     * @param q_Dot_Dot     output: joint acceleration: solve H q_Dot_Dot = torques - C(q,q_Dot) - g(q)
     * @return wrenches_ext optional input: external forces (not gravity)
     * @return 0 if successful
     */
    int getJointAcceleration(const std::vector<double>& q,
                             const std::vector<double>& q_Dot,
                             const std::vector<double>& torques,
                             std::vector<double>& q_Dot_Dot);

    /**
     * @brief getAcceleration gets the accerlaration for all moving frames
     * @param q joint positions
     * @param q_Dot joint velocities
     * @param q_DotDot joint accelerations
     * @param links name of the link frame
     * @param spatial_acc the spatial velocity of the link in link coordinates
     * @return
     */
    int getAcceleration(const std::vector<double>& q,
                        const std::vector<double>& q_Dot,
                        const std::vector<double>& q_DotDot,
                        std::vector<std::string>& links,
                        std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>> > &spatial_acc );
    /**
     * @brief getAcceleration gets the accerlaration for all moving frames
     * @param q joint positions
     * @param q_Dot joint velocities
     * @param q_DotDot joint accelerations
     * @param links name of the link frame
     * @param spatial_acc the spatial velocity of the link in link coordinates
     * @return
     */
    int getAcceleration(const std::vector<double>& q,
                        const std::vector<double>& q_Dot,
                        const std::vector<double>& q_DotDot, std::vector<std::string> &links,
                        std::vector<KDL::Twist > &spatial_acc );

    void changeDynamicParams(const std::string& link, const double mass, const Eigen::Vector3d& com, const Eigen::Matrix3d& inertia);
    void changeDynamicParams(const std::string& link, const double mass, const Eigen::Vector3d& com, const Eigen::Matrix3d& inertia, double gx, double gy, double gz);

    void changeKineticParams(const std::string& link, const Eigen::Vector3d& trans, const Eigen::Matrix3d& rotation);
    /**
     * @brief getChainDynParam calculates the matrices H,C and G needed for the inverse dynamics problem. KDL wrapper
     * @param q joint positions
     * @param q_Dot joint velocities
     * @param q_DotDot joint accelerations
     * @param H interia matrix
     * @param C coriolis and centripetal forces
     * @param G influnce of gravitaional forces
     * Wrapper for KDL::chainDynParam:
     * Implementation of a method to calculate the matrices H (inertia),C(coriolis) and G(gravitation) for the calculation torques out of the pose and derivatives. (inverse dynamics)
     * The algorithm implementation for H is based on the book "Rigid Body Dynamics Algorithms" of Roy Featherstone, 2008 (ISBN:978-0-387-74314-1) See page 107 for the pseudo-code. This algorithm is extended for the use of fixed joints
     * It calculates the joint-space inertia matrix, given the motion of the joints (q,qdot,qdotdot), external forces on the segments (expressed in the segments reference frame) and the dynamical parameters of the segments.
     *  torques  = H*qdotdot + C(q,qdot,f) + G(q)
     */
    int getChainDynParam(const std::vector<double>& q,
                         const std::vector<double>& q_Dot,
                         Eigen::MatrixXd& H, Eigen::VectorXd& C, Eigen::VectorXd& G);

    int getChainDynInertiaAndGravity(const std::vector<double>& q,
                                     Eigen::MatrixXd& H,
                                     Eigen::VectorXd& G);

    void useUrdfDynamicParams();

    double getLinkMass(const std::string &link) const;

    /**
     * @brief getLinkCoM returns the displacemant from link center of mass to origin
     * @param link the name of the link
     * @return displacement vector form link origin(rotation axis) to the center of mass.
     */
    Eigen::Vector3d getLinkCoM(const std::string &link) const;
    /**
     * @brief getLinkInertia gets the inertia moment matrix of a link
     * @param link the name of the link
     * @return the inertia moment matrix at link origin = rotation axis
     */
    Eigen::Matrix3d getLinkInertia(const std::string &link) const;

    /**
     * @brief getLinkInertiaCoM gets the inertia moment matrix of a link with respect to the center of mass;
     * @param link the name of the link
     * @return the inertia moment matrix at link origin = center of mass (CoM)
     */
    Eigen::Matrix3d getLinkInertiaCoM(const std::string &link) const;

    Eigen::Vector3d getURDFLinkCoM(const std::string &link) const;
    Eigen::Matrix3d getURDFLinkInertiaCoM(const std::string &link) const;
    double getURDFLinkMass(const std::string &link) const;

    Eigen::MatrixXd getRigidBodyRegressionMatrix(const std::string& root, const std::string& tip,
                                                 const std::vector<double>& q,
                                                 const std::vector<double>& q_Dot,
                                                 const std::vector<double>& q_DotDot,
                                                 const double& gx,
                                                 const double& gy,
                                                 const double& gz,
                                                 const bool project = true);

    Eigen::MatrixXd getStaticRigidBodyRegressionMatrix(const std::string &root,
                                                       const std::string &tip,
                                                       const std::vector<double> &q,
                                                       const double& gx,
                                                       const double& gy,
                                                       const double& gz,
                                                       const bool project = true);

    void modifiedRNE(const double gx,
                     const double gy,
                     const double gz,
                     const std::vector<double> &q1,
                     const std::vector<double> &q2,
                     const std::vector<double> &q3,
                     const std::vector<double> &q4,
                     Eigen::MatrixXd& res, std::size_t column = 0);

    void getMatrixC(const std::vector<double> &q,
                    const std::vector<double> &qDot,
                    Eigen::MatrixXd& res);

protected:
    KDL::Vector gravity_;
    std::shared_ptr<KDL::ChainIdSolver_RNE> solverID_;

    bool initialize() override;

    /**
     * @brief getLinkInertiaCoM gets the inertia moment matrix of a link with respect to the center of mass;
     * @param link the name of the link
     * @return the inertia moment matrix at link origin = center of mass (CoM)
     */
    KDL::RotationalInertia getLinkInertiaCoM(const int id) const;



};
}

#endif // CS_DynamicModel_H
