#ifndef CONTACT_POINT_HPP
#define CONTACT_POINT_HPP
#include <cslibs_kdl_data/header.h>
#include <cslibs_kdl_data/vector3.h>
namespace cslibs_kdl_data {
struct ContactPoint{
    Header header;
    double force = 1;
    Vector3 position;
    Vector3 direction;
};

}
#endif // CONTACT_POINT_HPP
