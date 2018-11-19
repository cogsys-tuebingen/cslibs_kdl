#ifndef CSLIBS_KDL_DATA_YAML_HPP
#define CSLIBS_KDL_DATA_YAML_HPP
#include <yaml-cpp/yaml.h>
#include <cslibs_kdl_data/accelerometer_data.h>
#include <cslibs_kdl_data/joint_state_data.h>
#include <cslibs_kdl_data/extended_joint_state_data.h>
#include <cslibs_kdl_data/joint_data.h>
#include <cslibs_kdl_data/wrench.h>
namespace YAML {

template<>
struct convert<cslibs_kdl_data::Header> {
    static Node encode(const cslibs_kdl_data::Header& rhs) {
        Node node;
        encode(node, rhs);
        return node;
    }

    static void encode(Node& node, const cslibs_kdl_data::Header& rhs) {
        node["frame_id"] = rhs.frame_id;
        node["stamp"] = rhs.stamp.toMicroSec();
    }

    static bool decode(const Node& node, cslibs_kdl_data::Header& rhs) {
        auto name = node["frame_id"];
        if(!name.IsDefined()) {
            return false;
        }
        rhs.frame_id = name.as<std::string>();

        auto stamp = node["stamp"];
        if(!stamp.IsDefined()){
            return false;
        }
        rhs.stamp.fromMicroSec(stamp.as<unsigned long int>());

        return true;
    }
};

template<>
struct convert<cslibs_kdl_data::Vector3> {
    static Node encode(const cslibs_kdl_data::Vector3& rhs) {
        Node node;

        encode(node,rhs);

        return node;
    }

    static void encode(Node& node, const cslibs_kdl_data::Vector3& rhs) {
        node["x"] = rhs.vector[0];
        node["y"] = rhs.vector[1];
        node["z"] = rhs.vector[2];
    }

    static bool decode(const Node& node, cslibs_kdl_data::Vector3& rhs) {
        auto nodeX = node["x"];
        if(!nodeX.IsDefined()){
            return false;
        }
        if(nodeX.Scalar() == "-nan" || nodeX.Scalar() == "nan"){   // broken force x sensor!
            rhs.vector[0] = std::numeric_limits<double>::quiet_NaN();
        } else {
            rhs.vector[0] = nodeX.as<double>();
        }
        auto nodeY = node["y"];
        if(!nodeY.IsDefined()){
            return false;
        }
        if(nodeY.Scalar() == "-nan" || nodeY.Scalar() == "nan"){ // broken force x sensor!
            rhs.vector[1] = std::numeric_limits<double>::quiet_NaN();
        } else {
            rhs.vector[1] = nodeY.as<double>();
        }
        auto nodeZ = node["z"];
        if(!nodeZ.IsDefined()){
            return false;
        }
        if(nodeZ.Scalar() == "-nan" || nodeZ.Scalar() == "nan"){ // broken force x sensor!
            rhs.vector[2] = std::numeric_limits<double>::quiet_NaN();
        } else {
            rhs.vector[2] = nodeZ.as<double>();
        }
        return true;
    }
};

template<>
struct convert<cslibs_kdl_data::Wrench> {
    static Node encode(const cslibs_kdl_data::Wrench& rhs) {
        Node node;

        encode(node,rhs);

        return node;
    }

    static void encode(Node& node, const cslibs_kdl_data::Wrench& rhs) {
        node["torque"] = rhs.torque;
        node["force"] = rhs.force;
    }

    static bool decode(const Node& node, cslibs_kdl_data::Wrench& rhs) {
        auto torque = node["torque"];
        if(!torque.IsDefined()){
            return false;
        }
        rhs.torque = torque.as<cslibs_kdl_data::Vector3>();
        auto force = node["force"];
        if(!force.IsDefined()){
            return false;
        }
        rhs.force = force.as<cslibs_kdl_data::Vector3>();
        return true;
    }
};


template<>
struct convert<cslibs_kdl_data::Vector3Stamped> {
    static Node encode(const cslibs_kdl_data::Vector3Stamped& rhs) {
        Node node = convert<cslibs_kdl_data::Header>::encode(rhs.header);
        convert<cslibs_kdl_data::Vector3>::encode(node, rhs.data);

        return node;
    }

    static bool decode(const Node& node, cslibs_kdl_data::Vector3Stamped& rhs) {
        if(!convert<cslibs_kdl_data::Header>::decode(node,rhs.header)){
            return false;
        }
        if(!convert<cslibs_kdl_data::Vector3>::decode(node,rhs.data)){
            return false;
        }
        return true;
    }
};

template<>
struct convert<cslibs_kdl_data::JointData>{
    static Node encode(const cslibs_kdl_data::JointData& rhs) {
        Node node;
        node["data"] = rhs.data;
        return node;
    }
    static bool decode(const Node& node, cslibs_kdl_data::JointData& rhs) {
        auto vec = node["data"];
        if(!vec.IsDefined()){
            return false;
        }
        rhs.data = vec.as<std::vector<double>>();
        return true;
    }
};

template<>
struct convert<cslibs_kdl_data::JointStateData> {
    static Node encode(const cslibs_kdl_data::JointStateData& rhs) {
        Node node;
        encode(node, rhs);
        return node;
    }

    static void encode(Node& node, const cslibs_kdl_data::JointStateData& rhs) {
        node["label"] = rhs.label;
        node["names"] = rhs.names;
        node["position"] = rhs.position;
        node["velocity"] = rhs.velocity;
        node["acceleration"] = rhs.acceleration;
        node["effort"] = rhs.torque;
        node["gravity"] = rhs.gravity2std();
    }

    static bool decode(const Node& node, cslibs_kdl_data::JointStateData& rhs) {
        auto name = node["names"];
        if(!name.IsDefined())
        {
            return false;
        }
        rhs.names = name.as<std::vector<std::string> >();
        auto pos = node["position"];
        if(!pos.IsDefined())
        {
            return false;
        }
        rhs.position = pos.as<std::vector<double> >();
        auto vel = node["velocity"];
        if(!vel.IsDefined())
        {
            return false;
        }
        rhs.velocity = vel.as<std::vector<double> >();
        auto acc = node["acceleration"];
        if(!acc.IsDefined())
        {
            return false;
        }
        rhs.acceleration = acc.as<std::vector<double> >();
        auto eff = node["effort"];
        if(!eff.IsDefined())
        {
            return false;
        }
        rhs.torque = eff.as<std::vector<double> >();
        YAML::Node label = node["label"];
        if(!label.IsDefined()) {
            return false;
        }
        rhs.label = label.as<int>();
        auto g = node["gravity"];
        if(g.IsDefined()){
            std::vector<double> vec = g.as<std::vector<double>>();
            rhs.setGravityFrom(vec);
        }
        return true;
    }
};

template<>
struct convert<cslibs_kdl_data::JointStateDataStamped> {
    static Node encode(const cslibs_kdl_data::JointStateDataStamped& rhs) {
        Node node = convert<cslibs_kdl_data::Header>::encode(rhs.header);
        convert<cslibs_kdl_data::JointStateData>::encode(node, rhs.data);
        return node;
    }

    static bool decode(const Node& node, cslibs_kdl_data::JointStateDataStamped& rhs) {
        if(!convert<cslibs_kdl_data::Header>::decode(node,rhs.header)){
            return false;
        }
        if(!convert<cslibs_kdl_data::JointStateData>::decode(node,rhs.data)){
            return false;
        }
        return true;
    }
};

template<>
struct convert<cslibs_kdl_data::AccelerometerData> {
    static Node encode(const cslibs_kdl_data::AccelerometerData& rhs) {
        Node node;
        node["label"] = rhs.label;
        for( cslibs_kdl_data::Vector3Stamped msg : rhs) {
            YAML::Node pNode;
            pNode["name"] = msg.header.frame_id;
            pNode["stamp"] = msg.header.stamp.toMicroSec();
            pNode["x"] = msg.data.vector[0];
            pNode["y"] = msg.data.vector[1];
            pNode["z"] = msg.data.vector[2];
            node["accelerometer"].push_back(pNode);
        }
        return node;
    }

    static bool decode(const Node& node, cslibs_kdl_data::AccelerometerData& rhs) {
        auto tmp = node["accelerometer"];
        if(!tmp.IsDefined())
        {
            return false;
        }
        for(auto it = tmp.begin(); it != tmp.end(); ++it){
            auto pNode = *it;
            cslibs_kdl_data::Vector3Stamped tmp;
            std::string name(pNode["name"].as<std::string>());
            tmp.header.frame_id = name;
            tmp.header.stamp.fromMicroSec(pNode["stamp"].as<unsigned long int>());
            tmp.data.vector[0] = pNode["x"].as<double>();
            tmp.data.vector[1] = pNode["y"].as<double>();
            tmp.data.vector[2] = pNode["z"].as<double>();
            rhs.push_back(tmp);
        }
        auto label = node["label"];
        if(!label.IsDefined())
        {
            return false;
        }
        rhs.label = label.as<int>();
        return true;
    }
};

template<>
struct convert<cslibs_kdl_data::ExtendedJointStateData> {
    static Node encode(const cslibs_kdl_data::ExtendedJointStateData& rhs) {
        Node node;
        node["Acclerometers"] = rhs.lin_acc;
        node["JointState"] = rhs.joint_state;
        return node;
    }

    static bool decode(const Node& node, cslibs_kdl_data::ExtendedJointStateData& rhs) {
        auto acc = node["Acclerometers"];
        if(!acc.IsDefined() || !acc.IsMap()) {
            return false;
        }
        rhs.lin_acc = acc.as<cslibs_kdl_data::AccelerometerData>();

        auto jstate = node["JointState"];
        if(!jstate.IsDefined() || !jstate.IsMap()){
            return false;
        }
        rhs.joint_state = jstate.as<cslibs_kdl_data::JointStateData>();
        return true;
    }
};

}
#endif // CSLIBS_KDL_DATA_YAML_HPP
