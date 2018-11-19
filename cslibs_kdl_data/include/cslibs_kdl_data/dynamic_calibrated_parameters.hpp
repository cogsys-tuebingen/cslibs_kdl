#ifndef CS_DYNAMIC_CALIBRATED_PARAMETERS_HPP
#define CS_DYNAMIC_CALIBRATED_PARAMETERS_HPP
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/StdVector>
namespace cslibs_kdl_calibration {

struct EIGEN_ALIGN16 DynamicParameters
{
    std::string linkName;
    double mass;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d coM;
    Eigen::Matrix3d inertia;

};

typedef std::vector<cslibs_kdl_calibration::DynamicParameters, Eigen::aligned_allocator<Eigen::Matrix3d>> DynamicParametersCollection;


inline void to_vector(const cslibs_kdl_calibration::DynamicParametersCollection& in, std::vector<double>& out, std::vector<std::string>& links,
                      bool move_mass = true, bool move_coM = true, bool move_inertia = true)
{
    out.clear();
    links.clear();
    for(auto param : in)
    {
        links.push_back(param.linkName);
        if(move_mass) {
            out.push_back(param.mass);
        }
        if(move_coM) {
            out.push_back(param.coM(0));
            out.push_back(param.coM(1));
            out.push_back(param.coM(2));
        }
        if(move_inertia) {
            out.push_back(param.inertia(0,0));
            out.push_back(param.inertia(0,1));
            out.push_back(param.inertia(0,2));
            out.push_back(param.inertia(1,1));
            out.push_back(param.inertia(1,2));
            out.push_back(param.inertia(2,2));
        }
    }

}

inline void to_Jaco2ManipulatorDynParams(const std::vector<double>& in, const std::vector<std::string> links,
                                         cslibs_kdl_calibration::DynamicParametersCollection& out,
                                         bool move_mass = true, bool move_coM = true, bool move_inertia = true)
{
   int elements_per_link = 0;
   if(move_mass) {
       elements_per_link += 1;
   }
   if(move_coM) {
       elements_per_link += 3;
   }
   if(move_inertia) {
       elements_per_link += 6;
   }
   if(links.size() != in.size() / elements_per_link)
   {
       throw std::logic_error("dimension mismatch between double vector and link name vector");
   }
   out.clear();
   int link_counter = 0;
   std::vector<double> values(elements_per_link);
   for(std::size_t i = 0; i < in.size(); ++i) {
       int local_id = i % elements_per_link;
       values[local_id] = in[i];
       if( i > 0 &&  local_id == elements_per_link - 1) {
           DynamicParameters param;
           param.linkName = links[link_counter];
           ++link_counter;
           switch (elements_per_link) {
           case 1:
               param.mass = values[0];
               break;
           case 3:
               param.coM = Eigen::Vector3d(values[0], values[1], values[2]);
               break;
           case 4:
               param.mass = values[0];
               param.coM = Eigen::Vector3d(values[1], values[2], values[3]);
               break;
           case 6:
               param.inertia << values[0], values[1], values[2],
                                values[1], values[3], values[4],
                                values[2], values[4], values[5];
               break;
           case 7:
               param.mass = values[0];
               param.inertia << values[1], values[2], values[3],
                                values[2], values[4], values[5],
                                values[2], values[5], values[6];
               break;
           case 9:
               param.coM = Eigen::Vector3d(values[0], values[1], values[2]);
               param.inertia << values[3], values[4], values[5],
                                values[4], values[6], values[7],
                                values[5], values[7], values[8];
               break;
           case 10:
               param.mass = values[0];
               param.coM = Eigen::Vector3d(values[1], values[2], values[3]);
               param.inertia << values[4], values[5], values[6],
                                values[5], values[7], values[8],
                                values[6], values[8], values[9];
               break;
           default:
               throw std::logic_error("something terribly wrong");
               break;
           }
           out.push_back(param);
       }
   }
}

inline void to_eigen(const cslibs_kdl_calibration::DynamicParametersCollection& in, Eigen::MatrixXd & out,
                      bool move_mass = true, bool move_coM = true, bool move_inertia = true)
{
//    int counter = 0;
    std::size_t sz = 0;
    if(move_mass){
        sz += 1;
    }
    if(move_coM){
        sz += 3;
    }
    if(move_inertia){
        sz+= 6;
    }
    out = Eigen::MatrixXd::Zero(in.size()*sz,1);
    for(std::size_t i = 0; i < in.size(); ++i) {
        auto param = in[i];
        int id = i*sz;
        switch (sz) {
        case 1:
            out(id) = param.mass;
            break;
        case 3:
            out.block<3,1>(id,0) = param.mass * param.coM;
            break;
        case 4:
            out(id) = param.mass;
            out.block<3,1>(id+1,0) = param.mass * param.coM;
            break;
        case 6:
            out(id,0) = param.inertia(0,0);
            out(id+1,0) = param.inertia(0,1);
            out(id+2,0) = param.inertia(0,2);
            out(id+3,0) = param.inertia(1,1);
            out(id+4,0) = param.inertia(1,2);
            out(id+5,0) = param.inertia(2,2);
            break;
        case 7:
            out(id) = param.mass;
            out(id+1,0) = param.inertia(0,0);
            out(id+2,0) = param.inertia(0,1);
            out(id+3,0) = param.inertia(0,2);
            out(id+4,0) = param.inertia(1,1);
            out(id+5,0) = param.inertia(1,2);
            out(id+6,0) = param.inertia(2,2);
            break;
        case 9:
            out.block<3,1>(id,0) = param.mass * param.coM;
            out(id+3,0) = param.inertia(0,0);
            out(id+4,0) = param.inertia(0,1);
            out(id+5,0) = param.inertia(0,2);
            out(id+6,0) = param.inertia(1,1);
            out(id+7,0) = param.inertia(1,2);
            out(id+8,0) = param.inertia(2,2);
            break;
        case 10:
            out(id) = param.mass;
            out.block<3,1>(id+1,0) = param.mass * param.coM;
            out(id+4,0) = param.inertia(0,0);
            out(id+5,0) = param.inertia(0,1);
            out(id+6,0) = param.inertia(0,2);
            out(id+7,0) = param.inertia(1,1);
            out(id+8,0) = param.inertia(1,2);
            out(id+9,0) = param.inertia(2,2);
            break;
        default:
            throw std::logic_error("something terribly wrong");
            break;
        }
    }
}


inline void to_Jaco2ManipulatorDynParams(const Eigen::MatrixXd & in, const std::vector<std::string> links,
                                         cslibs_kdl_calibration::DynamicParametersCollection& out)
{
    out.clear();
    int nlinks = in.size() / 10;
    for(int n = 0; n < nlinks; ++n) {
        DynamicParameters param;
        param.linkName = links[n];
        param.mass = in(n*10);
        param.coM = in.block<3,1>(n*10+1,0)/param.mass;
        Eigen::Matrix<double, 6, 1> inertia = in.block<6,1>(n*10+4,0);
        param.inertia << inertia(0), inertia(1), inertia(2),
                         inertia(1), inertia(3), inertia(4),
                         inertia(2), inertia(4), inertia(5);

        out.push_back(param);

    }
}

inline std::vector<std::string> getLinkNames(const cslibs_kdl_calibration::DynamicParametersCollection& params)
{
    std::vector<std::string> result;
    for(auto param : params) {
        result.push_back(param.linkName);
    }
    return result;
}


}
#endif // CS_DYNAMIC_CALIBRATED_PARAMETERS_HPP

