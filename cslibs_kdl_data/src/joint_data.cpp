#include <cslibs_kdl_data/joint_data.h>

using namespace cslibs_kdl_data;

JointData::JointData(std::size_t n)
{
    resize(n);
}

JointData::iterator JointData::begin()
{
    return data.begin();
}

JointData::const_iterator JointData::begin() const
{
    return data.begin();
}

JointData::iterator JointData::end()
{
    return data.end();
}

JointData::const_iterator JointData::end() const
{
    return data.end();
}

double& JointData::at(std::size_t i)
{
    return data.at(i);
}

const double& JointData::at(std::size_t i) const
{
    return data.at(i);
}

double& JointData::operator[](std::size_t i)
{
    return data[i];
}

const double& JointData::operator [](std::size_t i) const
{
    return data[i];
}

double& JointData::front()
{
    return data.front();
}

const double& JointData::front() const
{
    return data.front();
}

double& JointData::back()
{
    return data.back();
}

const double& JointData::back() const
{
    return data.back();
}

std::size_t JointData::size() const
{
    return data.size();
}
void JointData::resize(std::size_t n, double val)
{
    data.resize(n,val);
}

//void JointData::emplace_back(double&& val)
//{
//    data.emplace_back(val);
//}

void JointData::push_back(const double& val)
{
    data.push_back(val);
}

JointData JointData::operator+(const JointData &other) const
{
    JointData res;
    res.data.resize(this->data.size());
    auto it_other = other.data.begin();
    auto it_res = res.data.begin();
    for(const double& d : this->data){
        *it_res = d + *it_other;
        ++it_other;
        ++it_res;
    }
    return res;
}

JointData JointData::operator-(const JointData &other) const
{
    JointData res;
    res.data.resize(this->data.size());
    auto it_other = other.data.begin();
    auto it_res = res.data.begin();
    for(const double& d : this->data){
        *it_res = d - *it_other;
        ++it_other;
        ++it_res;
    }
    return res;
}
JointData JointData::operator*(const double &val) const
{
    JointData res;
    res.data.resize(this->data.size());
    auto it_res = res.data.begin();
    for(const double& d : this->data){
        *it_res = d * val;
        ++it_res;
    }
    return res;
}
JointData JointData::operator/(const double &val) const
{
    JointData res;
    res.data.resize(this->data.size());
    auto it_res = res.data.begin();
    for(const double& d : this->data){
        *it_res = d / val;
        ++it_res;
    }
    return res;
}

JointData& JointData::operator+=(const JointData &other)
{
    auto it_other = other.data.begin();
    for(double& d : this->data){
        d += *it_other;
        ++it_other;

    }
    return *this;
}

JointData& JointData::operator*=(const double &b)
{
    for(double& d : this->data){
        d *= d;

    }

    return *this;
}

JointData& JointData::operator/=(const double &b)
{
    for(double& d : this->data){
        d /= b;
    }

    return *this;
}
