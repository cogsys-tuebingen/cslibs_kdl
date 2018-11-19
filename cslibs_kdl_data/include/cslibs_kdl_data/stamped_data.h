#ifndef CS_STAMPED_DATA_H
#define CS_STAMPED_DATA_H
#include <cslibs_kdl_data/header.h>
namespace cslibs_kdl_data{
template<typename T>
class StampedData : public T
{
public:
    StampedData():
        data(*this)
    {}

    StampedData(const StampedData<T>& other):
        T(other), data(*this)
    {
        this->header = other.header;
        this->data = other.data;
    }

    StampedData(StampedData<T>&& other):
        T(other), data(*this)
    {
        header = std::move(other.header);
        data = std::move(other.data);
    }

    explicit StampedData(const T& d) :
        data(*this)
    {
        data = d ;
    }

    explicit StampedData(cslibs_kdl_data::Header& h, const T& d) :
        header(h),
        data(*this)
    {
        data = d;
    }

    explicit StampedData(T&& v)
        : data(*this)
    {
        data = std::move(v);
    }

    StampedData& operator = (const StampedData<T> & copy)
    {
        header = copy.header;
        data = copy.data;
        return *this;
    }
    StampedData& operator = (StampedData<T>&& copy)
    {
        header = std::move(copy.header);
        data = std::move(copy.data);
        return *this;
    }

    StampedData& operator = (const T & other_value)
    {
        data = other_value;
        return *this;
    }
    StampedData& operator = (T && other_value)
    {
        data = std::move(other_value);
        return *this;
    }

    TimeStamp stamp() const
    {
        return header.stamp;
    }
    TimeStamp& stamp()
    {
        return header.stamp;
    }

    std::string frameId() const
    {
        return header.frame_id;
    }

    std::string& frameId()
    {
        return header.frame_id;
    }

    StampedData<T> operator+(const StampedData<T> &other) const
    {
        StampedData<T> res;
        res.header.stamp.fromNSec(0.5* (this->header.stamp.toNSec() + other.header.stamp.toNSec()));
        res.header.frame_id = other.header.frame_id;
        res.data = this->data + other.data;
        return res;
    }

    StampedData<T> operator-(const StampedData<T> &other) const
    {
        StampedData<T> res;
        res.header.stamp.fromNSec(0.5* (this->header.stamp.toNSec() - other.header.stamp.toNSec()));
        res.header.frame_id = other.header.frame_id;
        res.data = this->data - other.data;
        return res;
    }

    StampedData<T> operator*(const double& b) const
    {
        StampedData<T> res;
        res.header  = header;
        res.data = this->data / b;
        return res;
    }

    StampedData<T> operator/(const double& b) const
    {
        StampedData<T> res;
        res.header  = header;
        res.data = this->data / b;
        return res;
    }

    StampedData<T>& operator+=(const StampedData<T> &other)
    {
        this->header.stamp.fromNSec(0.5* (this->header.stamp.toNSec() + other.header.stamp.toNSec()));
        this->data += other.data;
        return *this;
    }

    StampedData<T>& operator*=(const double &b)
    {
        this->data *= b;
    }

    StampedData<T>& operator/=(const double &b)
    {
        this->data /= b;
    }

    std::string toString(std::string delimiter = std::string(";")) const
    {
        std::stringstream ss;
        ss << std::to_string(header.stamp.toNSec()) + delimiter;
        ss << data.toString(delimiter);
        return ss.str();
    }

public:
    Header header;
    T& data;
};
}
#endif // CS_STAMPED_DATA_H
