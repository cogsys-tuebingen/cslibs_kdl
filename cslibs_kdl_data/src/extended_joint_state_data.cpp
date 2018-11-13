#include <cslibs_kdl_data/extended_joint_state_data.h>
using namespace cslibs_kdl_data;

ExtendedJointStateData::ExtendedJointStateData(std::size_t nj, std::size_t na)
    : joint_state(nj),
      lin_acc(na)
{
}

std::shared_ptr<std::vector<double>> ExtendedJointStateData::dataAccess(int type) const
{
    switch (type) {
    case (int) DataType::JOINT_POS:
        return std::make_shared<std::vector<double>>(joint_state.position);
        break;
    case (int) DataType::JOINT_VEL:
        return std::make_shared<std::vector<double>>(joint_state.velocity);
        break;
    case (int) DataType::JOINT_ACC:
        return std::make_shared<std::vector<double>>(joint_state.acceleration);
        break;
    case (int) DataType::JOINT_TORQUE:
        return std::make_shared<std::vector<double>>(joint_state.torque);
        break;
    case (int) DataType::LIN_ACC:
        return std::make_shared<std::vector<double>>(lin_acc.toVector());
    default:
        return nullptr;
        break;
    }
}

ExtendedJointStateData ExtendedJointStateData::abs() const
{
    ExtendedJointStateData res;
    res.joint_state = this->joint_state.abs();
    res.lin_acc = this->lin_acc.abs();
    return res;
}

std::string ExtendedJointStateData::to_string(const std::string delimiter) const
{
    std::stringstream ss;
    ss << joint_state.label << delimiter;
    for(auto val : joint_state.position){
        ss << val << delimiter;
    }
    for(auto val : joint_state.velocity){
        ss << val << delimiter;
    }
    for(auto val : joint_state.acceleration){
        ss << val << delimiter;
    }
    for(auto val : joint_state.torque){
        ss << val << delimiter;
    }
    for(auto acc : lin_acc)
    {
        ss << acc.data.vector[0] << delimiter << acc.data.vector[1] << delimiter << acc.data.vector[2] << delimiter;
    }
    return ss.str();
}

void ExtendedJointStateData::setLabel(int i)
{
    joint_state.label = i;
    lin_acc.label = i;
}

std::vector<double>& ExtendedJointStateData::position()
{
    return joint_state.position;
}

const std::vector<double>& ExtendedJointStateData::position() const
{
    return joint_state.position;
}

std::vector<double>& ExtendedJointStateData::velocity()
{
    return joint_state.velocity;
}

const std::vector<double> &ExtendedJointStateData::velocity() const
{
    return joint_state.velocity;
}


std::vector<double>& ExtendedJointStateData::acceleration()
{
    return joint_state.acceleration;
}

const std::vector<double>& ExtendedJointStateData::acceleration() const
{
    return joint_state.acceleration;
}

std::vector<double>& ExtendedJointStateData::torque()
{
    return joint_state.torque;
}

const std::vector<double> &ExtendedJointStateData::torque() const
{
    return joint_state.torque;
}

ExtendedJointStateData ExtendedJointStateData::operator+(const ExtendedJointStateData &other) const
{
    ExtendedJointStateData res;
    res.joint_state = this->joint_state + other.joint_state;
    res.lin_acc = this->lin_acc + other.lin_acc;
    return res;
}

ExtendedJointStateData ExtendedJointStateData::operator-(const ExtendedJointStateData &other) const
{
    ExtendedJointStateData res;
    res.joint_state = this->joint_state - other.joint_state;
    res.lin_acc = this->lin_acc - other.lin_acc;
    return res;
}

ExtendedJointStateData ExtendedJointStateData::operator*(const double &other) const
{
    ExtendedJointStateData res;
    res.joint_state = this->joint_state * other;
    res.lin_acc = this->lin_acc * other;
    return res;
}
ExtendedJointStateData ExtendedJointStateData::operator/(const double &other) const
{
    ExtendedJointStateData res;
    res.joint_state = this->joint_state / other;
    res.lin_acc = this->lin_acc / other;
    return res;
}

ExtendedJointStateData& ExtendedJointStateData::operator+=(const ExtendedJointStateData &other)
{
    this->joint_state += other.joint_state;
    this->lin_acc += other.lin_acc;
    return *this;
}

ExtendedJointStateData& ExtendedJointStateData::operator*=(const double &b)
{
    this->joint_state *= b;
    this->lin_acc *=b;
    return *this;
}

ExtendedJointStateData& ExtendedJointStateData::operator/=(const double &b)
{
    this->joint_state /= b;
    this->lin_acc /=b;
    return *this;
}

std::vector<std::string>& ExtendedJointStateData::names()
{
    return joint_state.names;
}

const std::vector<std::string>& ExtendedJointStateData::names() const
{
    return joint_state.names;
}
