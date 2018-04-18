#ifndef _ROS_turtlebot_euclid_msgs_Drive_h
#define _ROS_turtlebot_euclid_msgs_Drive_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace turtlebot_euclid_msgs
{

  class Drive : public ros::Msg
  {
    public:
      float drivers[2];
      enum { LEFT = 0 };
      enum { RIGHT = 1 };

    Drive():
      drivers()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 2; i++){
      union {
        float real;
        uint32_t base;
      } u_driversi;
      u_driversi.real = this->drivers[i];
      *(outbuffer + offset + 0) = (u_driversi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_driversi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_driversi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_driversi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->drivers[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 2; i++){
      union {
        float real;
        uint32_t base;
      } u_driversi;
      u_driversi.base = 0;
      u_driversi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_driversi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_driversi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_driversi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->drivers[i] = u_driversi.real;
      offset += sizeof(this->drivers[i]);
      }
     return offset;
    }

    const char * getType(){ return "turtlebot_euclid_msgs/Drive"; };
    const char * getMD5(){ return "e29d312aa28d0f158c71cf210161f39e"; };

  };

}
#endif