#include <cslibs_kdl_data/time_stamp.h>

using namespace cslibs_kdl_data;


TimeStamp::TimeStamp() :
  stamp(std::chrono::high_resolution_clock::now())
{
}

TimeStamp::TimeStamp(const TimeStamp &t):
  stamp(t.stamp)
{
}

TimeStamp::TimeStamp(TimeStamp &&t):
  stamp(t.stamp)
{
}

void TimeStamp::now()
{
    stamp = std::chrono::high_resolution_clock::now();
}

unsigned long int TimeStamp::toNSec() const
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count();
}

unsigned long int TimeStamp::toMicroSec() const
{
    return std::chrono::duration_cast<std::chrono::microseconds>(stamp.time_since_epoch()).count();
}

double TimeStamp::toSec() const
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count() * 1e-9;
}

void TimeStamp::fromNSec(unsigned long int nsecs)
{
    std::chrono::nanoseconds  tmp = std::chrono::nanoseconds(nsecs);
    stamp = std::chrono::time_point<std::chrono::high_resolution_clock>(tmp);
}

void TimeStamp::fromMicroSec(unsigned long int musecs)
{
    std::chrono::microseconds  tmp = std::chrono::microseconds(musecs);
    stamp = std::chrono::time_point<std::chrono::high_resolution_clock>(tmp);
}

void TimeStamp::fromSec(double sec)
{
    unsigned long int nsec = sec *1e9;
    std::chrono::nanoseconds  tmp = std::chrono::nanoseconds(nsec);
    stamp = std::chrono::time_point<std::chrono::high_resolution_clock>(tmp);
}

bool TimeStamp::operator !=(const TimeStamp& other) const
{
    return stamp != other.stamp;
}

bool TimeStamp::operator <=(const TimeStamp& other) const
{
    return stamp <= other.stamp;
}

bool TimeStamp::operator >=(const TimeStamp& other) const
{
    return stamp >= other.stamp;
}

bool TimeStamp::operator ==(const TimeStamp& other) const
{
    return stamp == other.stamp;
}

double TimeStamp::substractionResultInSeconds(const TimeStamp &rhs) const
{
    auto delta = stamp - rhs.stamp;
    double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(delta).count()*1e-9;
    return dt;
}

double TimeStamp::timeDiffinSeconds(const TimeStamp &lhs, const TimeStamp &rhs)
{
    auto delta = lhs.stamp - rhs.stamp;
    double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(delta).count()*1e-9;
    return dt;
}
