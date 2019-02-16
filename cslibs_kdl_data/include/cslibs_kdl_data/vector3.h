#ifndef CS_VECTOR3STAMPED_H
#define CS_VECTOR3STAMPED_H
#include <vector>
#include <Eigen/Dense>
#include <cslibs_kdl_data/time_stamp.h>

namespace cslibs_kdl_data {

class Vector3
{
public:
    Vector3();

    Vector3(double val);

    Vector3(double x, double y, double z);
    Vector3 operator+(const Vector3 &other) const;
    Vector3 operator-(const Vector3 &other) const;

    Vector3& operator+=(const Vector3 &other);
    Vector3& operator-=(const Vector3 &other);
    Vector3 operator*(const double &b) const;
    Vector3& operator =(const Eigen::Vector3d &v);

    Vector3 abs() const;

    double& operator ()(std::size_t i);
    const double& operator ()(std::size_t i) const;

    double& x();
    const double& x() const;
    double& y();
    const double& y() const;
    double& z();
    const double& z() const;

    ///
    /// \brief operator * element wise multiplication
    /// \param other the other vector
    /// \return element wise product
    ///
    Vector3 operator*(const Vector3 &other) const;

    Vector3 operator/(const double &b) const;

    Vector3& operator*=(const double &b);
    Vector3& operator/=(const double &b);
    void zero();

    double norm() const;
    void normalize();
    std::vector<double> toVector() const;
    std::string to_string(const std::string delimiter = std::string(";")) const;
    Eigen::Vector3d toEigen() const;
    void fromEigen(const Eigen::Vector3d& v);
    ///
    /// \brief dot multipication/ scalar product
    /// \param other the other vector
    /// \return dot product
    ///
    double dot(const Vector3& other) const;
    double angleBetween(const Vector3& other) const;

public:
    std::array<double,3> vector;

};

inline Vector3 operator*(double val , const Vector3& v)
{
  return v * val;
}

}
#endif // CS_VECTOR3STAMPED_H
