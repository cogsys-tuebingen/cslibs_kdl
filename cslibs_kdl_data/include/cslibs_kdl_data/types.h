#ifndef CS_KDL_TYPES_H
#define CS_KDL_TYPES_H

#include <vector>
#include <deque>
#include <Eigen/Core>
#include <cslibs_kdl_data/stamped_data.h>

// FORWARD DECLARATIONS

//namespace std
//{
//template <typename _Tp, typename _Alloc = std::allocator<_Tp>>
//class deque;
//}

namespace cslibs_kdl_data
{
class JointStateData;
class Vector3;
class Wrench;
class ExtendedJointStateData;
class AccelerometerData;
class JointData;
class JointAngles;
}


// TYPEDEFS
typedef Eigen::aligned_allocator<Eigen::Vector3d> EV3dAllocator;

namespace cslibs_kdl_data
{
typedef StampedData<JointStateData> JointStateDataStamped;
typedef StampedData<Vector3> Vector3Stamped;
typedef StampedData<Wrench> WrenchStamped;
typedef StampedData<ExtendedJointStateData> ExtendedJointStateDataStamped;
typedef StampedData<JointData> JointDataStamped;
typedef StampedData<JointAngles> JointAnglesStamped;

typedef std::vector<Vector3> Vector3Collection;
typedef std::vector<Vector3Stamped> Vector3StampedCollection;
typedef std::vector<JointStateData> JointStateDataCollection;
typedef std::vector<JointStateDataStamped> JointStateDataStampedCollection;
typedef std::vector<ExtendedJointStateData> ExtendedJointStateCollection;
typedef std::vector<ExtendedJointStateDataStamped> ExtendedJointStateStampedCollection;
typedef std::vector<JointData> JointDataCollection;
typedef std::vector<JointDataStamped> JointDataStampedCollection;
typedef std::vector<JointAngles> JointAnglesCollection;
typedef std::vector<JointAnglesStamped> JointAnglesStampedCollection;
typedef std::vector<AccelerometerData> AccelerometerDataCollection;

typedef std::deque<JointStateData> JointStateDataDeque;
typedef std::deque<JointStateDataStamped> JointStateDataStampedDeque;
typedef std::deque<ExtendedJointStateData> ExtendedJointStateDeque;
typedef std::deque<ExtendedJointStateDataStamped> ExtendedJointStateStampedDeque;
typedef std::deque<AccelerometerData> AccelerometerDataDeque;
}


#endif
