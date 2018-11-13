#ifndef DYNAMIC_CALIBRATION_IO_H
#define DYNAMIC_CALIBRATION_IO_H
#include <cslibs_kdl_data/dynamic_calibrated_parameters.hpp>

namespace cslibs_kdl_calibration {
class DynCalibrationIO{
public:
    DynCalibrationIO() {}

    static void save(std::string name, const DynamicParametersCollection &params);
    static void loadDynParm(std::string filename, DynamicParametersCollection& params);
};
}
#endif // DYNAMIC_CALIBRATION_IO_H
