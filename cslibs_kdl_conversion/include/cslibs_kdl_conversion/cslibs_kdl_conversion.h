#ifndef CSLIBS_KDL_CONVERSION_H
#define CSLIBS_KDL_CONVERSION_H
#include <cslibs_kdl_data/vector3.h>
#include <kdl/frames.hpp>
namespace cslibs_kdl {
    KDL::Vector convert(const cslibs_kdl_data::Vector3& v);

}
#endif // CSLIBS_KDL_CONVERSION_H
