#ifndef CS_WRENCH_H
#define CS_WRENCH_H
#include <cslibs_kdl_data/vector3.h>
#include <cslibs_kdl_data/stamped_data.h>
namespace cslibs_kdl_data {
class Wrench
{
    friend class StampedData<Wrench>;
public:
    Wrench();

    Wrench(double val);

    Wrench(double tx, double ty, double tz, double fx, double fy, double fz);
    Wrench(Eigen::Matrix<double, 6, 1> vec);
    Wrench(Vector3 torque, Vector3 force);
    Wrench operator+(const Wrench &other) const;
    Wrench operator-(const Wrench &other) const;

    Wrench& operator+=(const Wrench &other);
    Wrench& operator-=(const Wrench &other);
    Wrench operator*(const double &b) const;

    Wrench abs() const;

    ///
    /// \brief operator * element wise multiplication
    /// \param other the other vector
    /// \return element wise product
    ///
    Wrench operator*(const Wrench &other) const;

    Wrench operator/(const double &b) const;

    Wrench operator*=(const double &b);
    Wrench operator/=(const double &b);
    Eigen::Matrix<double, 6, 1> toEigen() const;

    double norm() const;
    std::vector<double> toVector() const;
    std::string to_string(std::string delimiter = std::string(";")) const;

public:
    Vector3 torque;
    Vector3 force;
};
}
#endif // CS_WRENCH_H
