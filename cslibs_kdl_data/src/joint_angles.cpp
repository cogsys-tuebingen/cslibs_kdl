#include <cslibs_kdl_data/joint_angles.h>
#include <math.h>
using namespace cslibs_kdl_data;

JointAngles::JointAngles()
    : normalized_(false)
{

}

JointAngles::JointAngles(std::size_t n)
    : JointData(n),
       normalized_(false)
{

}

JointAngles::JointAngles(const JointData &d)
    : JointData(d),
      normalized_(false)
{

}


JointAngles JointAngles::normalize() const
{
    JointAngles res;
    res.normalized_ = true;
    for(auto val : data){
        res.push_back(normalize(val));
    }
    return res;
}

double JointAngles::normalize(double angle)
{
    double result = angle;
    while(result < -M_PI){
        result += 2.0 * M_PI;
    }
    while(result > M_PI){
        result -= 2.0 * M_PI;
    }
    return result;
}

JointAngles& JointAngles::operator =(const JointData &other)
{
    this->data = other.data;
}

