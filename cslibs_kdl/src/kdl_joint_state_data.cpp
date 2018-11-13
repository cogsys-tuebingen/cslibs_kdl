#include <cslibs_kdl/kdl_joint_state_data.h>
#include <cslibs_kdl/kdl_conversion.h>
using namespace cslibs_kdl;
using namespace cslibs_kdl_data;

KDLJointStateData::KDLJointStateData():
    gravity(0,0,-9.81)
{

}

KDLJointStateData::KDLJointStateData(std::size_t n) :
    gravity(0,0,-9.81),
    position(n),
    velocity(n),
    acceleration(n),
    torque(n)

{
}

KDLJointStateData::KDLJointStateData(const JointStateData &data, std::size_t ignore_end) :
    label(data.label),
    gravity(data.gravity(0),data.gravity(1),data.gravity(2))
{
    convert(data.position, position, ignore_end);
    convert(data.velocity, velocity, ignore_end);
    convert(data.acceleration, acceleration, ignore_end);
    convert(data.torque, torque, ignore_end);
    for(std::size_t i = 0; i < data.names.size() - ignore_end; ++i){
        names.push_back(data.names[i]);
    }

}

void KDLJointStateData::resize(std::size_t n)
{
    position.resize(n);
    velocity.resize(n);
    acceleration.resize(n);
    torque.resize(n);
    setToZero();
}

void KDLJointStateData::fromManipulatorData(const JointStateData &data, std::size_t ignore_end)
{
    convert(data.position, position, ignore_end);
    convert(data.velocity, velocity, ignore_end);
    convert(data.acceleration, acceleration, ignore_end);
    convert(data.torque, torque, ignore_end);
    label = data.label;
    gravity = KDL::Vector(data.gravity(0),data.gravity(1),data.gravity(2));
    for(std::size_t i = 0; i < data.names.size() - ignore_end; ++i){
        names.push_back(data.names[i]);
    }
}

JointStateData KDLJointStateData::toJointStateData()
{
    JointStateData res;
    res.label = label;
    res.names = names;
    res.gravity = Eigen::Vector3d(gravity(0),gravity(1),gravity(2));
    convert(position, res.position);
    convert(velocity, res.velocity);
    convert(acceleration, res.acceleration);
    convert(torque, res.torque);
    return res;
}


void KDLJointStateData::setToZero()
{
    SetToZero(position);
    SetToZero(velocity);
    SetToZero(acceleration);
    SetToZero(torque);
}
