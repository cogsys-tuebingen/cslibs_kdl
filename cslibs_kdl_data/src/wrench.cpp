#include <cslibs_kdl_data/wrench.h>
using namespace cslibs_kdl_data;
Wrench::Wrench() :
    torque(0,0,0),
    force(0,0,0)

{

}

Wrench::Wrench(double val):
    torque(val,val,val),
    force(val,val,val)
{

}

Wrench::Wrench(double tx, double ty, double tz, double fx, double fy, double fz):
    torque(tx,ty,tz),
    force(fx,fy,fz)
{

}

Wrench::Wrench(Eigen::Matrix<double, 6, 1> vec) :
    torque(vec(0), vec(1), vec(2)),
    force(vec(3), vec(4), vec(5))
{

}

Wrench::Wrench(Vector3 _torque, Vector3 _force):
    torque(_torque),
    force(_force)
{
}

Wrench Wrench::operator+(const Wrench &other) const
{
    Wrench res;
    res.torque = this->torque + other.torque;
    res.force = this->force + other.force;
    return res;
}
Wrench Wrench::operator-(const Wrench &other) const
{
    Wrench res;
    res.torque = this->torque - other.torque;
    res.force = this->force - other.force;
    return res;
}

Wrench& Wrench::operator+=(const Wrench &other)
{
    this->torque +=  other.torque;
    this->force  += other.force;
    return *this;
}

Wrench& Wrench::operator-=(const Wrench &other)
{
    this->torque -=  other.torque;
    this->force  -= other.force;
    return *this;
}

Wrench Wrench::operator*(const double &b) const
{
    Wrench res;
    res.torque = this->torque * b;
    res.force = this->force * b;
    return res;
}

Wrench Wrench::abs() const
{
    Wrench res;
    res.torque = this->torque.abs();
    res.force = this->force.abs();
    return res;
}

Wrench Wrench::operator*(const Wrench &other) const
{
    Wrench res;
    res.torque = this->torque * other.torque;
    res.force = this->force * other.force;
    return res;
}

Wrench Wrench::operator/(const double &b) const
{
    Wrench res;
    res.torque = this->torque / b;
    res.force = this->force / b;
    return res;
}


Wrench Wrench::operator*=(const double &b)
{
    this->torque *= b;
    this->force  *= b;
    return *this;
}

Wrench Wrench::operator/=(const double &b)
{
    this->torque /= b;
    this->force  /= b;
    return *this;
}
Eigen::Matrix<double, 6, 1> Wrench::toEigen() const
{
    Eigen::Matrix<double, 6, 1> res;
    res << torque(0), torque(1), torque(2),
            force(0),  force(1), force(2);
    return res;

}

double Wrench::norm() const
{
    return toEigen().norm();
}

std::vector<double> Wrench::toVector() const
{
    std::vector<double> res  = {torque(0), torque(1), torque(2),
                                force(0),  force(1), force(2)};
    return res;
}

std::string Wrench::to_string(const std::string delimiter) const
{
    return torque.to_string(delimiter) + delimiter + force.to_string(delimiter);
}
