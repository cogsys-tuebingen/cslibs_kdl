#ifndef CS_TIME_STAMP_H
#define CS_TIME_STAMP_H
#include <chrono>
namespace cslibs_kdl_data {

class TimeStamp
{
public:
    TimeStamp();
    TimeStamp(const TimeStamp& t);
    TimeStamp(TimeStamp&& t);

    TimeStamp& operator = (const TimeStamp &other)
    {
        stamp = other.stamp;
        return *this;
    }

    TimeStamp& operator = (TimeStamp &&other)
    {
        stamp = other.stamp;
        return *this;
    }


    void now();

    unsigned long int toNSec() const;
    unsigned long int toMicroSec() const;
    double toSec() const;

    void fromNSec(unsigned long int nsecs);
    void fromMicroSec(unsigned long int musecs);
    void fromSec(double sec);

    bool operator !=(const TimeStamp& other) const;
    bool operator <=(const TimeStamp& other) const;
    bool operator >=(const TimeStamp& other) const;
    bool operator ==(const TimeStamp& other) const;

    double substractionResultInSeconds(const TimeStamp& rhs) const;

    static double timeDiffinSeconds(const TimeStamp& lhs, const TimeStamp& rhs);


public:
    std::chrono::time_point<std::chrono::high_resolution_clock> stamp;
};

}
#endif // CS_TIME_STAMP_H
