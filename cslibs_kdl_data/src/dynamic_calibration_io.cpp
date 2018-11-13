#include <cslibs_kdl_data/dynamic_calibration_io.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>
using namespace cslibs_kdl_calibration;
void DynCalibrationIO::save(std::string name, const DynamicParametersCollection &params)
{
    std::ofstream file(name);
    YAML::Emitter yamlEmit(file);
    YAML::Node doc;
    for(DynamicParameters param : params)
    {
        YAML::Node pNode;
        pNode["link_name"] = param.linkName;
        pNode["mass"] = param.mass;
        Eigen::Vector3d com = param.coM;
        pNode["com_x"] = com(0);
        pNode["com_y"] = com(1);
        pNode["com_z"] = com(2);
        Eigen::Matrix3d inertia = param.inertia;
        pNode["Ixx"] = inertia(0,0);
        pNode["Ixy"] = inertia(0,1);
        pNode["Ixz"] = inertia(0,2);
        pNode["Iyy"] = inertia(1,1);
        pNode["Iyz"] = inertia(1,2);
        pNode["Izz"] = inertia(2,2);
        doc["parameter"].push_back(pNode);
    }
    yamlEmit << doc;

}


void DynCalibrationIO::loadDynParm(std::string filename, DynamicParametersCollection& params)
{
    YAML::Node doc = YAML::LoadFile(filename);
    doc = doc["parameter"];
    if(!doc.IsDefined())
    {
        throw std::runtime_error("illegal document!");
    }
    for(auto it = doc.begin(); it != doc.end(); ++it){
        auto pNode = *it;
        DynamicParameters param;
        std::string name(pNode["link_name"].as<std::string>());
        param.linkName = name;
        param.mass = pNode["mass"].as<double>();
        double x = pNode["com_x"].as<double>();
        double y = pNode["com_y"].as<double>();
        double z = pNode["com_z"].as<double>();
        param.coM = Eigen::Vector3d(x, y, z);
        double ixx = pNode["Ixx"].as<double>();
        double ixy = pNode["Ixy"].as<double>();
        double ixz = pNode["Ixz"].as<double>();
        double iyy = pNode["Iyy"].as<double>();
        double iyz = pNode["Iyz"].as<double>();
        double izz = pNode["Izz"].as<double>();
        param.inertia << ixx, ixy, ixz,
                         ixy, iyy, iyz,
                         ixz, iyz, izz;
        params.push_back(param);
    }
}
