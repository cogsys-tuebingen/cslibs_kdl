#ifndef EIGEN_YAML_H
#define EIGEN_YAML_H
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace YAML {
template<>
struct convert<Eigen::MatrixXd> {
    static Node encode(const Eigen::MatrixXd& rhs) {
        Node node;
        node["rows"] = rhs.rows();
        node["cols"] = rhs.cols();
        for(std::size_t i = 0; i < (std::size_t) rhs.rows(); ++i){
            for(std::size_t j = 0; j < (std::size_t) rhs.cols(); ++j){
                node["data"].push_back(rhs(i,j));
            }
        }
        return node;
    }

    static bool decode(const Node& node, Eigen::MatrixXd& rhs) {

        auto nrows = node["rows"];
        if(!nrows.IsDefined()){
            return false;
        }
        std::size_t rows = nrows.as<std::size_t>();

        auto ncols = node["cols"];
        if(!ncols.IsDefined()){
            return false;
        }
        std::size_t cols = ncols.as<std::size_t>();

        rhs = Eigen::MatrixXd::Zero(rows, cols);

        auto data = node["data"];
        if(!data.IsDefined()){
            return false;
        }
        if(!data.IsSequence() || data.size() != rows*cols) {
            return false;
        }
        for(std::size_t i = 0; i < (std::size_t) rhs.rows(); ++i){
            for(std::size_t j = 0; j < (std::size_t) rhs.cols(); ++j){
                std::size_t id = i*cols + j;
                rhs(i,j) = data[id].as<double>();
            }
        }

        return true;
    }
};

template<>
struct convert<Eigen::VectorXd> {
    static Node encode(const Eigen::VectorXd& rhs) {
        Node node;
        node["rows"] = rhs.rows();
        node["cols"] = rhs.cols();
        for(std::size_t i = 0; i < (std::size_t) rhs.rows(); ++i){
            for(std::size_t j = 0; j < (std::size_t) rhs.cols(); ++j){
                node["data"].push_back(rhs(i,j));
            }
        }
        return node;
    }

    static bool decode(const Node& node, Eigen::VectorXd& rhs) {

        auto nrows = node["rows"];
        if(!nrows.IsDefined()){
            return false;
        }
        std::size_t rows = nrows.as<std::size_t>();

        auto ncols = node["cols"];
        if(!ncols.IsDefined()){
            return false;
        }
        std::size_t cols = ncols.as<std::size_t>();

        rhs = Eigen::VectorXd::Zero(rows, cols);

        auto data = node["data"];
        if(!data.IsDefined()){
            return false;
        }
        if(!data.IsSequence() || data.size() != rows*cols) {
            return false;
        }
        for(std::size_t i = 0; i < (std::size_t) rhs.rows(); ++i){
            for(std::size_t j = 0; j < (std::size_t) rhs.cols(); ++j){
                std::size_t id = i*cols + j;
                rhs(i,j) = data[id].as<double>();
            }
        }

        return true;
    }
};

template<>
struct convert<Eigen::VectorXi> {
    static Node encode(const Eigen::VectorXi& rhs) {
        Node node;
        node["rows"] = rhs.rows();
        node["cols"] = rhs.cols();
        for(std::size_t i = 0; i < (std::size_t) rhs.rows(); ++i){
            for(std::size_t j = 0; j < (std::size_t) rhs.cols(); ++j){
                node["data"].push_back(rhs(i,j));
            }
        }
        return node;
    }

    static bool decode(const Node& node, Eigen::VectorXi& rhs) {

        auto nrows = node["rows"];
        if(!nrows.IsDefined()){
            return false;
        }
        std::size_t rows = nrows.as<std::size_t>();

        auto ncols = node["cols"];
        if(!ncols.IsDefined()){
            return false;
        }
        std::size_t cols = ncols.as<std::size_t>();

        rhs = Eigen::VectorXi::Zero(rows, cols);

        auto data = node["data"];
        if(!data.IsDefined()){
            return false;
        }
        if(!data.IsSequence() || data.size() != rows*cols) {
            return false;
        }
        for(std::size_t i = 0; i < (std::size_t) rhs.rows(); ++i){
            for(std::size_t j = 0; j < (std::size_t) rhs.cols(); ++j){
                std::size_t id = i*cols + j;
                rhs(i,j) = data[id].as<int>();
            }
        }

        return true;
    }
};

template<>
struct convert<Eigen::ArrayXd> {
    static Node encode(const Eigen::ArrayXd& rhs) {
        Node node;
        node["rows"] = rhs.rows();
        node["cols"] = rhs.cols();
        for(std::size_t i = 0; i < (std::size_t) rhs.rows(); ++i){
            for(std::size_t j = 0; j < (std::size_t) rhs.cols(); ++j){
                node["data"].push_back(rhs(i,j));
            }
        }
        return node;
    }

    static bool decode(const Node& node, Eigen::ArrayXd& rhs) {

        auto nrows = node["rows"];
        if(!nrows.IsDefined()){
            return false;
        }
        std::size_t rows = nrows.as<std::size_t>();

        auto ncols = node["cols"];
        if(!ncols.IsDefined()){
            return false;
        }
        std::size_t cols = ncols.as<std::size_t>();

        rhs = Eigen::ArrayXd::Zero(rows, cols);

        auto data = node["data"];
        if(!data.IsDefined()){
            return false;
        }
        if(!data.IsSequence() || data.size() != rows*cols) {
            return false;
        }
        for(std::size_t i = 0; i < (std::size_t) rhs.rows(); ++i){
            for(std::size_t j = 0; j < (std::size_t) rhs.cols(); ++j){
                std::size_t id = i*cols + j;
                rhs(i,j) = data[id].as<double>();
            }
        }

        return true;
    }
};
}
#endif // EIGEN_YAML_H
