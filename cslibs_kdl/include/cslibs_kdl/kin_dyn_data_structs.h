#ifndef CS_KIN_DYN_DATA_STRUCTS_H
#define CS_KIN_DYN_DATA_STRUCTS_H
#include <vector>

namespace cslibs_kdl {

struct ResidualData
{
    double dt;
    double gx;
    double gy;
    double gz;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> torques;

    inline void resize(std::size_t n){
        joint_positions.resize(n);
        joint_velocities.resize(n);
        torques.resize(n);
    }
};

struct JointState{

    double dt;
    double gx;
    double gy;
    double gz;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_accelerations;
    std::vector<double> torques;

    ResidualData getResidualData()
    {
        ResidualData res;
        res.dt = dt;
        res.gx = gx;
        res.gy = gy;
        res.gz = gz;
        res.joint_positions = joint_positions;
        res.joint_velocities = joint_velocities;
        res.torques = torques;
        return res;
    }

};


}
#endif // CS_KIN_DYN_DATA_STRUCTS_H

