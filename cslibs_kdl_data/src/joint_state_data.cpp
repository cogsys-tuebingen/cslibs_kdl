#include <cslibs_kdl_data/joint_state_data.h>

using namespace cslibs_kdl_data;

JointStateData::JointStateData() :
    label(-1),
    gravity(0,0,0)
{
}

JointStateData::JointStateData(std::size_t n) :
    label(-1),
    gravity(0,0,0)
{
    resize(n);
}

void JointStateData::resize(std::size_t n, double val)
{
    names.resize(n,"");
    position.resize(n, val);
    velocity.resize(n, val);
    acceleration.resize(n, val);
    torque.resize(n, val);
}

void JointStateData::normalize(std::size_t offset )
{
    std::size_t i = 0;
    for(auto it = position.begin(); it < position.end() - offset; ++it){
        if(i == 1 || i == 2){
            while(*it >= 2.0*M_PI){
                *it -= 2.0*M_PI;
            }
            while(*it < 0){
                *it += 2.0 *M_PI;
            }
        } else{
            while(*it > M_PI){
                *it -= 2.0*M_PI;
            }
            while(*it <= -M_PI){
                *it += 2.0 *M_PI;
            }
        }
        ++i;
    }
}

JointStateData::iterator JointStateData::begin(DataType type)
{
    switch (type) {
    case DataType::JOINT_POS:
        return position.begin();
        break;
    case DataType::JOINT_VEL:
        return velocity.begin();
        break;
    case DataType::JOINT_ACC:
        return acceleration.begin();
        break;
    case DataType::JOINT_TORQUE:
        return torque.begin();
        break;
    }
}

JointStateData::const_iterator JointStateData::begin(DataType type) const
{
    switch (type) {
    case DataType::JOINT_POS:
        return position.begin();
        break;
    case DataType::JOINT_VEL:
        return velocity.begin();
        break;
    case DataType::JOINT_ACC:
        return acceleration.begin();
        break;
    case DataType::JOINT_TORQUE:
        return torque.begin();
        break;
    }
}

JointStateData::iterator JointStateData::end(DataType type)
{
    switch (type) {
    case DataType::JOINT_POS:
        return position.end();
        break;
    case DataType::JOINT_VEL:
        return velocity.end();
        break;
    case DataType::JOINT_ACC:
        return acceleration.end();
        break;
    case DataType::JOINT_TORQUE:
        return torque.end();
        break;
    }
}
JointStateData::const_iterator JointStateData::end(DataType type) const
{
    switch (type) {
    case DataType::JOINT_POS:
        return position.end();
        break;
    case DataType::JOINT_VEL:
        return velocity.end();
        break;
    case DataType::JOINT_ACC:
        return acceleration.end();
        break;
    case DataType::JOINT_TORQUE:
        return torque.end();
        break;
    }
}

Eigen::VectorXd JointStateData::getEigenVector(DataType type, std::size_t offset) const
{
    const std::vector<double>* data;
    switch (type) {
    case DataType::JOINT_POS:
        data = &position;
        break;
    case DataType::JOINT_VEL:
        data = &velocity;
        break;
    case DataType::JOINT_ACC:
        data = &acceleration;
        break;
    case DataType::JOINT_TORQUE:
        data = &torque;
        break;
    }
    return convert2eigen(data, offset);
}

Eigen::VectorXd JointStateData::convert2eigen(const std::vector<double> *data, std::size_t offset)
{
    if(!data){
        return Eigen::VectorXd();
    }
    std::size_t size = data->size() - offset;
    Eigen::VectorXd res(size);
    auto it = data->begin();
    for(std::size_t i = 0; i < size; ++i){
        res(i) = *it;
        ++it;
    }
    return res;

}

JointStateData JointStateData::abs() const
{
    JointStateData res = *this;

    auto it = res.position.begin();
    for(auto p : res.position)
    {
        *it = fabs(p);
        ++it;
    }

    it = res.velocity.begin();
    for(auto p : res.velocity)
    {
        *it = fabs(p);
        ++it;
    }

    it = res.acceleration.begin();
    for(auto p : res.acceleration)
    {
        *it = fabs(p);
        ++it;
    }

    it = res.torque.begin();
    for(auto p : res.torque)
    {
        *it = fabs(p);
        ++it;
    }

    return res;
}


double JointStateData::norm(int type) const
{
    return norm((DataType) type);
}

double JointStateData::norm(DataType type) const
{
    const std::vector<double>* data;
    switch (type) {
    case DataType::JOINT_POS:
        data = &position;
        break;
    case DataType::JOINT_VEL:
        data = &velocity;
        break;
    case DataType::JOINT_ACC:
        data = &acceleration;
        break;
    case DataType::JOINT_TORQUE:
        data = &torque;
        break;
    default:
        data = nullptr;
        break;
    }

    double norm = 0;
    if(data){
        for(double val : *data){
            norm += val*val;
        }
        norm = sqrt(norm);
    }
    return norm;
}

JointStateData JointStateData::operator+(const JointStateData &other) const
{
    JointStateData res(this->position.size());
    auto it_res = res.position.begin();
    auto it_other = other.position.begin();

    res.gravity = this->gravity + other.gravity;
    for(auto d : this->position){
        *it_res = d + *it_other;
        ++it_res;
        ++it_other;
    }

    it_res = res.velocity.begin();
    it_other = other.velocity.begin();

    for(auto d : this->velocity){
        *it_res = d + *it_other;
        ++it_res;
        ++it_other;
    }

    it_res = res.acceleration.begin();
    it_other = other.acceleration.begin();

    for(auto d : this->acceleration){
        *it_res = d + *it_other;
        ++it_res;
        ++it_other;
    }

    it_res = res.torque.begin();
    it_other = other.torque.begin();

    for(auto d : this->torque){
        *it_res = d + *it_other;
        ++it_res;
        ++it_other;
    }
    return res;

}

JointStateData JointStateData::operator-(const JointStateData &other) const
{
    JointStateData res(this->position.size());
    auto it_res = res.position.begin();
    auto it_other = other.position.begin();

    res.gravity = this->gravity - other.gravity;
    for(auto d : this->position){
        *it_res = d - *it_other;
        ++it_res;
        ++it_other;
    }

    it_res = res.velocity.begin();
    it_other = other.velocity.begin();

    for(auto d : this->velocity){
        *it_res = d - *it_other;
        ++it_res;
        ++it_other;
    }

    it_res = res.acceleration.begin();
    it_other = other.acceleration.begin();

    for(auto d : this->acceleration){
        *it_res = d - *it_other;
        ++it_res;
        ++it_other;
    }

    it_res = res.torque.begin();
    it_other = other.torque.begin();

    for(auto d : this->torque){
        *it_res = d - *it_other;
        ++it_res;
        ++it_other;
    }
    return res;

}

JointStateData JointStateData::operator*(const double &b) const
{
    JointStateData res(this->position.size());
    auto it_res = res.position.begin();
    res.gravity = this->gravity * b;

    for(auto d : this->position){
        *it_res = d * b;
        ++it_res;
    }

    it_res = res.velocity.begin();

    for(auto d : this->velocity){
        *it_res = d * b;
        ++it_res;
    }

    it_res = res.acceleration.begin();

    for(auto d : this->acceleration){
        *it_res = d * b;
        ++it_res;
    }

    it_res = res.torque.begin();

    for(auto d : this->torque){
        *it_res = d * b;
        ++it_res;
    }
    return res;
}

JointStateData JointStateData::operator/(const double &b) const
{
    JointStateData res(this->position.size());
    auto it_res = res.position.begin();
    res.gravity = this->gravity / b;

    for(auto d : this->position){
        *it_res = d / b;
        ++it_res;
    }

    it_res = res.velocity.begin();

    for(auto d : this->velocity){
        *it_res = d / b;
        ++it_res;
    }

    it_res = res.acceleration.begin();

    for(auto d : this->acceleration){
        *it_res = d / b;
        ++it_res;
    }

    it_res = res.torque.begin();

    for(auto d : this->torque){
        *it_res = d / b;
        ++it_res;
    }
    return res;
}

JointStateData& JointStateData::operator+=(const JointStateData &other)
{
    auto it_other = other.position.begin();

    this->gravity += other.gravity;
    for(auto it_p = this->position.begin();  it_p < this->position.end(); ++it_p){
        *it_p += *it_other;
        ++it_other;
    }

    it_other = other.velocity.begin();

    for(auto it_v = this->velocity.begin();  it_v < this->velocity.end(); ++it_v){
        *it_v += *it_other;
        ++it_other;
    }

    it_other = other.acceleration.begin();

    for(auto it_v = this->acceleration.begin();  it_v < this->acceleration.end(); ++it_v){
        *it_v += *it_other;
        ++it_other;
    }

    it_other = other.torque.begin();

    for(auto it_v = this->torque.begin();  it_v < this->torque.end(); ++it_v){
        *it_v += *it_other;
        ++it_other;
    }
    return *this;
}

JointStateData& JointStateData::operator*=(const double &b)
{
    this->gravity *= b;

    for(double& d  : this->position){
        d *= b;
    }

    for(double& d : this->velocity){
        d *= b;
    }

    for(double& d : this->acceleration){
        d *= b;
    }


    for(double& d : this->torque){
        d *= b;
    }
    return *this;
}

JointStateData& JointStateData::operator/=(const double &b)
{
    this->gravity /= b;

    for(double& d  : this->position){
        d /= b;
    }

    for(double& d : this->velocity){
        d /= b;
    }

    for(double& d : this->acceleration){
        d /= b;
    }


    for(double& d : this->torque){
        d /= b;
    }
    return *this;
}

std::string JointStateData::toString(std::string delimiter) const
{
    //    std::string res;
    std::stringstream ss;
    for(auto val : position)
    {
        ss << std::to_string(val) + delimiter;
    }
    for(auto val : velocity)
    {
        ss << std::to_string(val) + delimiter;
    }
    for(auto val : acceleration)
    {
        ss << std::to_string(val) + delimiter;
    }
    for(auto val : torque)
    {
        ss << std::to_string(val) + delimiter;
    }
    ss << std::to_string(gravity(0)) + delimiter +
          std::to_string(gravity(1)) + delimiter +
          std::to_string(gravity(2)) + delimiter;
    return ss.str();
}

void JointStateData::popToSize(std::size_t n)
{
    while(names.size() > n){
        names.pop_back();
    }
    while(position.size() > n){
        position.pop_back();
    }
    while(velocity.size() > n){
        velocity.pop_back();
    }
    while(acceleration.size() > n){
        acceleration.pop_back();
    }
    while(torque.size() > n){
        torque.pop_back();
    }
}

std::vector<double> JointStateData::gravity2std() const
{
    std::vector<double> res;
    res.push_back(gravity(0));
    res.push_back(gravity(1));
    res.push_back(gravity(2));
    return res;
}
void JointStateData::setGravityFrom(const std::vector<double>& data)
{
    gravity(0) = data.at(0);
    gravity(1) = data.at(1);
    gravity(2) = data.at(2);
}

