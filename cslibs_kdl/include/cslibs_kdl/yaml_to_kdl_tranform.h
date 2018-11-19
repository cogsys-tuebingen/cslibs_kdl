#ifndef CS_YAML_TO_TRANSFORM_CONVERSION_HPP
#define CS_YAML_TO_TRANSFORM_CONVERSION_HPP
#include <vector>
#include <cslibs_kdl_data/suppress_warnings_start.h>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <cslibs_kdl_data/suppress_warnings_end.h>
#include <yaml-cpp/yaml.h>

namespace cslibs_kdl{

struct KDLTransformation {
    KDLTransformation(){}

    std::string name;
    std::string parent;
    KDL::Frame frame;


};

void save(std::string name, const std::vector<KDLTransformation>& transforms);

void load(std::string filename, std::vector<KDLTransformation>& transforms);

void convert(YAML::Node& node, std::vector<KDLTransformation>& transforms);

void frames(const std::vector<KDLTransformation>& in, std::vector<KDL::Frame>& out);


}
#endif // CS_YAML_TO_TRANSFORM_CONVERSION_HPP

