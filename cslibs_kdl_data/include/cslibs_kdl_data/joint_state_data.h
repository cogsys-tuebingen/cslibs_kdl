#ifndef JOINT_STATE_DATA_H
#define JOINT_STATE_DATA_H
#include <vector>
#include <Eigen/Core>
#include <cslibs_kdl_data/types.h>
#include <cslibs_kdl_data/header.h>
#include <cslibs_kdl_data/vector3.h>

namespace cslibs_kdl_data {


class JointStateData
{
public:
    typedef std::vector<double>::iterator iterator;
    typedef std::vector<double>::const_iterator const_iterator;
public:
    enum class DataType{
        JOINT_POS = 1,
        JOINT_VEL = 2,
        JOINT_ACC = 3,
        JOINT_TORQUE = 4
    };

    JointStateData();
    JointStateData(std::size_t n);

    void resize(std::size_t n, double val = 0);
    // normalizes to Jaco2 model limits
    void normalize(std::size_t offset = 0);

    iterator begin(DataType type);
    const_iterator begin(DataType type) const;

    iterator end(DataType type);
    const_iterator end(DataType type) const;

    Eigen::VectorXd getEigenVector(DataType type, std::size_t offset = 0) const;

    JointStateData abs() const;

    double norm(int type) const;
    double norm(DataType type) const;

    JointStateData operator+(const JointStateData &other) const;
    JointStateData operator-(const JointStateData &other) const;
    JointStateData operator*(const double &b) const;
    JointStateData operator/(const double &b) const;
    JointStateData& operator+=(const JointStateData &other);
    JointStateData& operator*=(const double &b);
    JointStateData& operator/=(const double &b);

    std::string toString(std::string delimiter = std::string(";")) const;
    void popToSize(std::size_t n);
    std::vector<double> gravity2std() const;
    void setGravityFrom(const std::vector<double>& data);

private:
    static Eigen::VectorXd convert2eigen(const std::vector<double> *data, std::size_t offset = 0);

public:
    int label;
    cslibs_kdl_data::Vector3 gravity;
    std::vector<std::string> names;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> torque;
};
}
#endif // JOINT_STATE_DATA_H
