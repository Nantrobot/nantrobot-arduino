#ifndef _ROS_pixart_world_point_h
#define _ROS_pixart_world_point_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PointStamped.h"

namespace pixart
{

  class world_point : public ros::Msg
  {
    public:
      geometry_msgs::PointStamped pt;
      uint16_t camera_id;
      uint8_t point_id;

    world_point():
      pt(),
      camera_id(0),
      point_id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pt.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->camera_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->camera_id >> (8 * 1)) & 0xFF;
      offset += sizeof(this->camera_id);
      *(outbuffer + offset + 0) = (this->point_id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->point_id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pt.deserialize(inbuffer + offset);
      this->camera_id =  ((uint16_t) (*(inbuffer + offset)));
      this->camera_id |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->camera_id);
      this->point_id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->point_id);
     return offset;
    }

    const char * getType(){ return "pixart/world_point"; };
    const char * getMD5(){ return "4c1801fffad721cf6519e3411a1948c7"; };

  };

}
#endif