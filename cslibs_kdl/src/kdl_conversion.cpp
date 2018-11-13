#include <cslibs_kdl/kdl_conversion.h>

void cslibs_kdl::convert(const KDL::JntArray &in, std::vector<double> &out)
{
    out.resize(in.rows());
    for(std::size_t i = 0; i < out.size(); ++i){
        out[i] = in(i);
    }
}

void cslibs_kdl::convert(const std::vector<double> &in, KDL::JntArray &out, std::size_t ignore_end)
{
    out.resize(in.size() - ignore_end);
    for(std::size_t i = 0; i < out.rows(); ++i){
        out(i) = in[i];
    }
}

void cslibs_kdl::convert(const std::vector<double> &in, Eigen::VectorXd &out, std::size_t ignore_end)
{
    out.resize(in.size() - ignore_end);
    std::size_t i = 0;
    for(auto it = in.begin(); it < in.end() - ignore_end; ++it, ++i){
        out(i) = *it;
    }
}

void cslibs_kdl::convert(const Eigen::VectorXd &in, std::vector<double> &out, std::size_t ignore_end)
{
    out.resize(in.rows() -ignore_end);
    for(std::size_t i = 0; i < out.size(); ++i){
        out[i] = in(i);
    }
}

void cslibs_kdl::poseTFToKDL(const tf::Pose& t, KDL::Frame& k)
{
    for (unsigned int i = 0; i < 3; ++i){
        k.p[i] = t.getOrigin()[i];
    }
    for (unsigned int i = 0; i < 9; ++i){
        k.M.data[i] = t.getBasis()[i/3][i%3];
    }
}

Eigen::Matrix3d cslibs_kdl::skewSymMat(const KDL::Vector &vec)
{
    Eigen::Matrix3d res;
    res << 0     , -vec(2)   , vec(1),
           vec(2), 0         , -vec(0),
          -vec(1), vec(0)    , 0;
    return res;
}

Eigen::Matrix<double, 3, 6> cslibs_kdl::inertiaProductMat(const KDL::Vector &vec)
{
    Eigen::Matrix<double, 3, 6> res;
    res << vec(0), vec(1), vec(2), 0     , 0     , 0,
            0    , vec(0), 0     , vec(1), vec(2), 0,
            0    , 0     , vec(0), 0     , vec(1), vec(2);
    return res;

}

Eigen::Matrix<double, 6, 6> cslibs_kdl::convert2EigenTwistTransform(const KDL::Frame &frame)
{
    Eigen::Matrix<double, 6, 6> result;
    Eigen::Matrix<double, 3, 3> rot = convert2Eigen(frame.M);
    result.block<3,3>(0,0) = rot;
    result.block<3,3>(0,3).setZero();
    result.block<3,3>(3,0) = skewSymMat(frame.p) * rot;
    result.block<3,3>(3,3) = rot;
    return result;
}

Eigen::Matrix<double, 6, 6> cslibs_kdl::convert2EigenWrenchTransform(const KDL::Frame &frame)
{
    Eigen::Matrix<double, 6, 6> result;
    Eigen::Matrix<double, 3, 3> rot = convert2Eigen(frame.M);
    result.block<3,3>(0,0) = rot;
    result.block<3,3>(0,3) = skewSymMat(frame.p) * rot;
    result.block<3,3>(3,0).setZero();
    result.block<3,3>(3,3) = rot;
    return result;
}

Eigen::Matrix<double, 3, 3> cslibs_kdl::convert2Eigen(const KDL::Rotation &rot)
{
    Eigen::Matrix<double, 3, 3> result;
    result << rot.data[0], rot.data[1], rot.data[2],
            rot.data[3], rot.data[4], rot.data[5],
            rot.data[6], rot.data[7], rot.data[8];
    return result;
}

void cslibs_kdl::kdlJntArray2Eigen(const KDL::JntArray &q, Eigen::VectorXd& res)
{
    res.setZero(q.rows());
    for(std::size_t i = 0; i < q.rows(); ++i){
        res(i) = q(i);
    }
}

void cslibs_kdl::convert2Eigen(const KDL::JntSpaceInertiaMatrix& mat, Eigen::MatrixXd & res)
{
    res.setZero(mat.rows(), mat.columns());
    for(std::size_t i = 0; i < mat.rows(); ++i){
        for(std::size_t j = 0; j < mat.columns(); ++j){
            res(i,j) = mat(i,j);
        }
    }
}

Eigen::Matrix<double, 6, 1> cslibs_kdl::convert2Eigen(const KDL::Twist& twist)
{
    Eigen::Matrix<double, 6, 1> result;
    result << twist.rot(0), twist.rot(1), twist.rot(2),
              twist.vel(0), twist.vel(1), twist.vel(2);
    return result;

}

Eigen::Matrix<double, 6, 1> cslibs_kdl::convert2Eigen(const KDL::Wrench& wrench)
{
    Eigen::Matrix<double, 6, 1> result;
    result << wrench.torque(0), wrench.torque(1), wrench.torque(2),
              wrench.force(0), wrench.force(1), wrench.force(2);
    return result;

}

void cslibs_kdl::vectorKDLToEigen(const KDL::Vector &in, Eigen::Vector3d &out)
{
    out(0) = in(0);
    out(1) = in(1);
    out(2) = in(2);
}

void cslibs_kdl::rotationKDLToEigen(const KDL::Rotation &in, Eigen::Quaterniond &out)
{
    double x,y,z,w;
    in.GetQuaternion(x,y,z,w);
    out = Eigen::Quaterniond(w,x,y,z);
}

KDL::Wrench cslibs_kdl::convert(const cslibs_kdl_data::Wrench& w)
{
    KDL::Vector f(w.force(0), w.force(1), w.force(2));
    KDL::Vector t(w.torque(0), w.torque(1), w.torque(2));
    return KDL::Wrench(f, t);
}
