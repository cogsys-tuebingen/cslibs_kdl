#include <cslibs_kdl_conversion/cslibs_kdl_conversion.h>

using namespace cslibs_kdl;

KDL::Vector convert(const cslibs_kdl_data::Vector3& v)
{
    KDL::Vector res(v(0),v(1),v(2));
    return res;
}


void HeaderConversion::ros2data(const std_msgs::Header& in, cslibs_kdl_data::Header& out)
{
    out.frame_id = in.frame_id;
    out.stamp.fromNSec(in.stamp.toNSec());
}

void HeaderConversion::data2ros(const cslibs_kdl_data::Header &in, std_msgs::Header &out)
{
    out.frame_id = in.frame_id;
    out.stamp.fromNSec(in.stamp.toNSec());
}

void Vector3Conversion::ros2data(const geometry_msgs::Vector3& in, cslibs_kdl_data::Vector3& out)
{
    out(0) = in.x;
    out(1) = in.y;
    out(2) = in.z;
}

void Vector3StampedConversion::ros2data(const geometry_msgs::Vector3Stamped &in, cslibs_kdl_data::Vector3Stamped& out)
{
    HeaderConversion::ros2data(in.header, out.header);
    Vector3Conversion::ros2data(in.vector, out);
}

void JointStateConversion::ros2data(const sensor_msgs::JointState &in, cslibs_kdl_data::JointStateData &out)
{
    out.names    = in.name;
    out.position = in.position;
    out.velocity = in.velocity;
    out.torque   = in.effort;
}

void JointStateConversion::data2ros(const cslibs_kdl_data::JointStateData &in, sensor_msgs::JointState &out)
{
    out.name     = in.names;
    out.position = in.position;
    out.velocity = in.velocity;
    out.effort   = in.torque;
}

void JointStateStampedConversion::ros2data(const sensor_msgs::JointState &in, cslibs_kdl_data::JointStateDataStamped &out)
{
    HeaderConversion::ros2data(in.header, out.header);
    JointStateConversion::ros2data(in, out);
}

void JointStateStampedConversion::data2ros(const cslibs_kdl_data::JointStateDataStamped &in, sensor_msgs::JointState &out)
{
    HeaderConversion::data2ros(in.header, out.header);
    JointStateConversion::data2ros(in.data, out);
}
