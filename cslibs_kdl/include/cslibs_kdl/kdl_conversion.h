#ifndef KDL_CONVERSION_H
#define KDL_CONVERSION_H

#include <cslibs_kdl_data/suppress_warnings_start.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <Eigen/Core>
#include <tf/tf.h>
#include <cslibs_kdl_data/suppress_warnings_end.h>
#include <cslibs_kdl_data/wrench.h>

namespace cslibs_kdl {

     void convert(const KDL::JntArray& in, std::vector<double>& out);
     void convert(const std::vector<double>& in, KDL::JntArray& out, std::size_t ignore_end = 0);
     void convert(const std::vector<double> &in, Eigen::VectorXd &out, std::size_t ignore_end = 0 );
     void convert(const Eigen::VectorXd &in, std::vector<double> &out, std::size_t ignore_end = 0);
     void poseTFToKDL(const tf::Pose& t, KDL::Frame& k);
     Eigen::Matrix3d skewSymMat(const KDL::Vector& vec);
     Eigen::Matrix<double, 3, 6> inertiaProductMat(const KDL::Vector& vec);
     Eigen::Matrix<double, 6, 6> convert2EigenTwistTransform(const KDL::Frame& frame);
     Eigen::Matrix<double, 6, 6> convert2EigenWrenchTransform(const KDL::Frame& frame);
     Eigen::Matrix<double, 3, 3> convert2Eigen(const KDL::Rotation& rot);
     Eigen::Matrix<double, 6, 1> convert2Eigen(const KDL::Twist& twist);
     Eigen::Matrix<double, 6, 1> convert2Eigen(const KDL::Wrench& wrench);
     void kdlJntArray2Eigen(const KDL::JntArray& q, Eigen::VectorXd &res);
     void convert2Eigen(const KDL::JntSpaceInertiaMatrix& mat, Eigen::MatrixXd &res);

     void vectorKDLToEigen(const KDL::Vector& in, Eigen::Vector3d& out);
     void rotationKDLToEigen(const KDL::Rotation& in, Eigen::Quaterniond& out);

     KDL::Wrench convert(const cslibs_kdl_data::Wrench& w);




}
#endif // KDL_CONVERSION_H
