#include <iostream>
#include <fstream>
#include <cslibs_kdl/yaml_to_kdl_tranform.h>
#include <yaml-cpp/yaml.h>


namespace cslibs_kdl{


void save(std::string name, const std::vector<KDLTransformation>& transforms)
{
    std::ofstream file(name);
    YAML::Emitter yamlEmit(file);
    YAML::Node doc;
    for(KDLTransformation trans : transforms)
    {
        YAML::Node pNode;
        pNode["frame_name"] = trans.name;
        pNode["parent"] = trans.parent;
        pNode["p_x"] = trans.frame.p(0);
        pNode["p_y"] = trans.frame.p(1);
        pNode["p_z"] = trans.frame.p(2);

        double x,y,z,w;
        trans.frame.M.GetQuaternion(x,y,z,w);

        pNode["q_x"] = x;
        pNode["q_y"] = y;
        pNode["q_z"] = z;
        pNode["q_w"] = w;
        doc["parameter"].push_back(pNode);
    }
    yamlEmit << doc;

}

void load(std::string filename, std::vector<KDLTransformation>& transforms)
{
    YAML::Node doc = YAML::LoadFile(filename);
    doc = doc["parameter"];
    if(!doc.IsDefined())
    {
        throw std::runtime_error("illegal document!");
    }
    convert(doc, transforms);
}

void convert(YAML::Node& node, std::vector<KDLTransformation>& transforms)
{
    for(auto it = node.begin(); it != node.end(); ++it){
        auto pNode = *it;
        KDLTransformation param;
        std::string name(pNode["frame_name"].as<std::string>());
        param.name = name;
        std::string parent(pNode["parent"].as<std::string>());
        param.parent = parent;

        double p_x = pNode["p_x"].as<double>();
        double p_y = pNode["p_y"].as<double>();
        double p_z = pNode["p_z"].as<double>();
        param.frame.p = KDL::Vector(p_x, p_y, p_z);
        double q_x = pNode["q_x"].as<double>();
        double q_y = pNode["q_y"].as<double>();
        double q_z = pNode["q_z"].as<double>();
        double q_w = pNode["q_w"].as<double>();
        param.frame.M = KDL::Rotation::Quaternion(q_x,q_y,q_z,q_w);

        transforms.push_back(param);
    }
}

void frames(const std::vector<KDLTransformation> &in, std::vector<KDL::Frame> &out)
{

    out.resize(in.size());
    auto it = out.begin();
    for(auto n_transform : in) {
        *it = n_transform.frame;
        ++it;
    }
}

}
