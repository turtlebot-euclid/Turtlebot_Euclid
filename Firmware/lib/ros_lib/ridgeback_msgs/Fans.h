#ifndef _ROS_ridgeback_msgs_Fans_h
#define _ROS_ridgeback_msgs_Fans_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ridgeback_msgs
{

  class Fans : public ros::Msg
  {
    public:
      uint8_t fans[6];
      enum { EQUIPMENT_BAY_INTAKE = 0 };
      enum { EQUIPMENT_BAY_EXHAUST = 1 };
      enum { CHARGER_BAY_INTAKE = 2 };
      enum { CHARGER_BAY_EXHAUST = 3 };
      enum { USER_BAY_INTAKE = 4 };
      enum { USER_BAY_EXHAUST = 5 };
      enum { FAN_OFF = 0 };
      enum { FAN_ON_HIGH = 1 };
      enum { FAN_ON_LOW = 2 };

    Fans():
      fans()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 6; i++){
      *(outbuffer + offset + 0) = (this->fans[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fans[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 6; i++){
      this->fans[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->fans[i]);
      }
     return offset;
    }

    const char * getType(){ return "ridgeback_msgs/Fans"; };
    const char * getMD5(){ return "d529aec610975f8df12d912730064bbf"; };

  };

}
#endif