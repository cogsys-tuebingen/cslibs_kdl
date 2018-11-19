#ifndef CS_JOINT_DATA_H
#define CS_JOINT_DATA_H
#include <vector>
#include <string>
#include <cslibs_kdl_data/header.h>
namespace cslibs_kdl_data {

class JointData
{
public:
    typedef std::vector<double>::iterator iterator;
    typedef std::vector<double>::const_iterator const_iterator;
public:
    JointData() {}
    JointData(std::size_t n);
    iterator begin();
    const_iterator begin() const;

    iterator end();
    const_iterator end() const;

    double& at(std::size_t i);
    const double& at(std::size_t i) const;

    double& operator[](std::size_t i);
    const double& operator [](std::size_t i) const;

    double& front();
    const double& front() const;

    double& back();
    const double& back() const;

    std::size_t size() const;
    void resize(std::size_t n, double val = 0);

//    void emplace_back(double&& val);
    void push_back(const double& val);

    JointData operator+(const JointData &other) const;
    JointData operator-(const JointData &other) const;
    JointData operator*(const double &val) const;
    JointData operator/(const double &val) const;
    JointData& operator+=(const JointData &other);
    JointData& operator*=(const double &b);
    JointData& operator/=(const double &b);

public:
    std::vector<double> data;
};
}
#endif // JOINT_DATA_H
