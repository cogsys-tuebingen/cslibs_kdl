#ifndef CS_KDLJOINTSTATEDATA_H
#define CS_KDLJOINTSTATEDATA_H
#include <cslibs_kdl_data/suppress_warnings_start.h>
#include <kdl/jntarray.hpp>
#include <cslibs_kdl_data/suppress_warnings_end.h>
#include <cslibs_kdl_data/joint_state_data.h>
namespace cslibs_kdl {
class KDLJointStateData
{
public:
    enum class DataType{
        JOINT_POS = 1,
        JOINT_VEL = 2,
        JOINT_ACC = 3,
        JOINT_TORQUE = 4
    };

    KDLJointStateData();
    KDLJointStateData(std::size_t n);
    KDLJointStateData(const cslibs_kdl_data::JointStateData& data, std::size_t ignore_end = 0);
    void resize(std::size_t n);

    void fromManipulatorData(const cslibs_kdl_data::JointStateData& data, std::size_t ignore_end = 0);
    cslibs_kdl_data::JointStateData toJointStateData();

    void setToZero();

public:
    int label;
    KDL::Vector gravity;
    std::vector<std::string> names;
    KDL::JntArray position;
    KDL::JntArray velocity;
    KDL::JntArray acceleration;
    KDL::JntArray torque;
};
}
#endif // CS_KDLJOINTSTATEDATA_H
