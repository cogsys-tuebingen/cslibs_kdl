#include <cslibs_kdl_data/accelerometer_data.h>

using namespace cslibs_kdl_data;

AccelerometerData::AccelerometerData()
    : label(-1)
{}

AccelerometerData::AccelerometerData(std::size_t n)
    : label(-1)
{
    resize(n);
}

AccelerometerData::iterator AccelerometerData::AccelerometerData::begin()
{
    return lin_acc.begin();
}

AccelerometerData::const_iterator AccelerometerData::begin() const
{
    return lin_acc.begin();
}

AccelerometerData::iterator AccelerometerData::end()
{
    return lin_acc.end();
}

AccelerometerData::const_iterator AccelerometerData::end() const
{
    return lin_acc.end();
}


Vector3Stamped& AccelerometerData::at(std::size_t i)
{
    return lin_acc.at(i);
}

const Vector3Stamped& AccelerometerData::at(std::size_t i) const
{
    return lin_acc.at(i);
}

Vector3Stamped& AccelerometerData::operator[](std::size_t i)
{
    return lin_acc[i];
}

const Vector3Stamped& AccelerometerData::operator [](std::size_t i) const
{
    return lin_acc[i];
}

double& AccelerometerData::operator()(std::size_t i, std::size_t comp)
{
    return lin_acc[i].data(comp);
}

const double& AccelerometerData::operator ()(std::size_t i, std::size_t comp) const
{
    return lin_acc[i].data(comp);
}

std::size_t AccelerometerData::size() const
{
    return lin_acc.size();
}

void AccelerometerData::resize(std::size_t n, const Vector3Stamped &val)
{
    lin_acc.resize(n, val);
}

Vector3Stamped& AccelerometerData::front()
{
    return lin_acc.front();
}

const Vector3Stamped& AccelerometerData::front() const
{
    return lin_acc.front();
}

Vector3Stamped& AccelerometerData::back()
{
    return lin_acc.back();
}

const Vector3Stamped& AccelerometerData::back() const
{
    return lin_acc.back();
}

//void AccelerometerData::emplace_back(Vector3StampedStamped&& val)
//{
//    lin_acc.emplace_back(val);
//}

void AccelerometerData::push_back(const Vector3Stamped &val)
{
    lin_acc.push_back(val);
}

std::vector<double> AccelerometerData::toVector() const
{
    std::vector<double> res;
    for(const auto& v : lin_acc){
        std::vector<double> tmp = v.data.toVector();
        res.insert(res.end(),tmp.begin(), tmp.end());
    }
    return res;
}

AccelerometerData AccelerometerData::abs() const
{
    AccelerometerData res = *this;

    for(Vector3Stamped& p : res.lin_acc)
    {
        p = p.data.abs();
    }

    return res;
}

double AccelerometerData::norm() const
{
    double res = 0;

    for(std::size_t i = 0; i < this->lin_acc.size(); ++i)
    {
        res += this->lin_acc[i].data.norm();
    }

    return res;
}

AccelerometerData AccelerometerData::operator+(const AccelerometerData &other) const
{
    AccelerometerData res;
    res.resize(this->size());
    auto it = other.begin();
    auto it_res = res.begin();
    for(auto v : lin_acc){
        *it_res = v + *it;
        ++it_res;
        ++it;
    }

    return res;
}

AccelerometerData AccelerometerData::operator-(const AccelerometerData &other) const
{
    AccelerometerData res;
    res.resize(this->size());
    auto it = other.begin();
    auto it_res = res.begin();
    for(auto v : lin_acc){
        *it_res = v - *it;
        ++it_res;
        ++it;
    }

    return res;
}

AccelerometerData AccelerometerData::operator*(const double &b) const
{
    AccelerometerData res;
    res.resize(this->size());
    auto it_res = res.begin();
    for(const auto& v : lin_acc){
        *it_res = v  * b;
        ++it_res;
    }
    return res;
}

AccelerometerData AccelerometerData::operator/(const double &b) const
{
    AccelerometerData res;
    res.resize(this->size());
    auto it_res = res.begin();
    for(const auto& v : lin_acc){
        *it_res = v  / b;
        ++it_res;
    }
    return res;
}

AccelerometerData& AccelerometerData::operator+=(const AccelerometerData &other)
{
    auto it = other.begin();
    for(Vector3Stamped& v : lin_acc){
        v += *it;
        ++it;
    }
    return *this;
}

AccelerometerData& AccelerometerData::operator*=(const double &b)
{
    for(Vector3Stamped& v : lin_acc){
        v *= b;
    }
    return *this;
}

AccelerometerData& AccelerometerData::operator/=(const double &b)
{
    for(Vector3Stamped& v : lin_acc){
        v /= b;
    }
    return *this;
}
