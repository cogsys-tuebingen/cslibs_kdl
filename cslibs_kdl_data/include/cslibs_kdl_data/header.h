#ifndef CS_HEADER_H
#define CS_HEADER_H

#include <string>
#include <cslibs_kdl_data/time_stamp.h>
namespace cslibs_kdl_data
{

struct Header{

    Header() {}
    Header(const Header& h):
        stamp(h.stamp),
        frame_id(h.frame_id)
    {}

    Header(Header && h):
        stamp(h.stamp),
        frame_id(std::move(h.frame_id))
    {
    }

    Header(const std::string& frame_id_, const TimeStamp& stamp_) :
        stamp(stamp_),
        frame_id(frame_id_)
    {
    }

    Header& operator = (const Header &h)
    {
        stamp = h.stamp;
        frame_id = h.frame_id;
        return *this;
    }

    Header& operator = (Header &&h)
    {
        stamp = h.stamp;
        frame_id = std::move(h.frame_id);
        return *this;
    }



    TimeStamp stamp;
    std::string frame_id;
};
}
#endif // CS_HEADER_H
