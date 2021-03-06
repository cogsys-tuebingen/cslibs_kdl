#ifndef CSLIBS_KDL_CONVERSION_H
#define CSLIBS_KDL_CONVERSION_H
#include <cslibs_kdl_data/types.h>
#include <cslibs_kdl_data/vector3.h>
#include <cslibs_kdl_data/joint_state_data.h>
#include <cslibs_kdl_data/contact_point.hpp>
#include <cslibs_kdl_msgs/ContactMessage.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <kdl/frames.hpp>
namespace cslibs_kdl {
    KDL::Vector convert(const cslibs_kdl_data::Vector3& v);

    struct HeaderConversion
    {
        static void ros2data(const std_msgs::Header& in, cslibs_kdl_data::Header& out);
        static void data2ros(const cslibs_kdl_data::Header& in, std_msgs::Header& out);
    };

    struct Vector3Conversion
    {
        static void ros2data(const geometry_msgs::Vector3& in, cslibs_kdl_data::Vector3& out);
        static void data2ros(const cslibs_kdl_data::Vector3& in, geometry_msgs::Vector3& out);
        static void ros2data(const geometry_msgs::Point& in, cslibs_kdl_data::Vector3& out);
        static void data2ros(const cslibs_kdl_data::Vector3& in, geometry_msgs::Point& out);
    };
    struct Vector3StampedConversion
    {
        static void ros2data(const geometry_msgs::Vector3Stamped& in, cslibs_kdl_data::Vector3Stamped& out);
        static void data2ros(const cslibs_kdl_data::Vector3Stamped& in, geometry_msgs::Vector3Stamped& out);
    };

    struct JointStateConversion
    {
        static void ros2data(const sensor_msgs::JointState& in, cslibs_kdl_data::JointStateData& out);
        static void data2ros(const cslibs_kdl_data::JointStateData& in, sensor_msgs::JointState& out);
    };

    struct JointStateStampedConversion
    {
        static void ros2data(const sensor_msgs::JointState& in, cslibs_kdl_data::JointStateDataStamped& out);
        static void data2ros(const cslibs_kdl_data::JointStateDataStamped& in, sensor_msgs::JointState& out);
    };

    struct ContactPointConversion
    {
        static void ros2data(const cslibs_kdl_msgs::ContactMessage& in, cslibs_kdl_data::ContactPoint& out);
        static void data2ros(const cslibs_kdl_data::ContactPoint& in, cslibs_kdl_msgs::ContactMessage& out);
    };

}
#endif // CSLIBS_KDL_CONVERSION_H
