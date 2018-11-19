#ifndef CS_ACCELEROMETER_DATA_H
#define CS_ACCELEROMETER_DATA_H
#include <cslibs_kdl_data/types.h>
#include <vector>
#include <Eigen/StdVector>
#include <cslibs_kdl_data/vector3.h>
#include <cslibs_kdl_data/types.h>


namespace cslibs_kdl_data {

class AccelerometerData
{
public:
    typedef Vector3StampedCollection::iterator iterator;
    typedef Vector3StampedCollection::const_iterator const_iterator;

public:

    AccelerometerData();
    AccelerometerData(std::size_t n);

    iterator begin();
    const_iterator begin() const;

    iterator end();
    const_iterator end() const;

    Vector3Stamped& at(std::size_t i);
    const Vector3Stamped& at(std::size_t i) const;

    Vector3Stamped& operator[](std::size_t i);
    const Vector3Stamped& operator [](std::size_t i) const;

    double& operator()(std::size_t i, std::size_t comp);
    const double& operator ()(std::size_t i, std::size_t comp) const;

    Vector3Stamped& front();
    const Vector3Stamped& front() const;

    Vector3Stamped& back();
    const Vector3Stamped& back() const;

    std::size_t size() const;
    void resize(std::size_t n, const Vector3Stamped& val = Vector3Stamped(Vector3(0,0,0)));

//    void emplace_back(Vector3StampedStamped&& val);
    void push_back(const Vector3Stamped& val);

    std::vector<double> toVector() const;
    AccelerometerData abs() const;

    double norm() const;

    AccelerometerData operator+(const AccelerometerData &other) const;
    AccelerometerData operator-(const AccelerometerData &other) const;
    AccelerometerData operator*(const double &b) const;
    AccelerometerData operator/(const double &b) const;
    AccelerometerData& operator+=(const AccelerometerData &other);
    AccelerometerData& operator*=(const double &b);
    AccelerometerData& operator/=(const double &b);


public:
    int label;
private:
    Vector3StampedCollection lin_acc;
};
}
#endif // CS_ACCELEROMETER_DATA_H
