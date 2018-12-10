#include <cslibs_kdl_conversion/cslibs_kdl_conversion.h>

using namespace cslibs_kdl;

KDL::Vector convert(const cslibs_kdl_data::Vector3& v)
{
    KDL::Vector res(v(0),v(1),v(2));
    return res;
}
