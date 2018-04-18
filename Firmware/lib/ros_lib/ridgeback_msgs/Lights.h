#ifndef _ROS_ridgeback_msgs_Lights_h
#define _ROS_ridgeback_msgs_Lights_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ridgeback_msgs/RGB.h"

namespace ridgeback_msgs
{

  class Lights : public ros::Msg
  {
    public:
      ridgeback_msgs::RGB lights[8];
      enum { LIGHTS_FRONT_LEFT_UPPER = 0 };
      enum { LIGHTS_FRONT_LEFT_LOWER = 1 };
      enum { LIGHTS_FRONT_RIGHT_UPPER = 2 };
      enum { LIGHTS_FRONT_RIGHT_LOWER = 3 };
      enum { LIGHTS_REAR_LEFT_UPPER = 4 };
      enum { LIGHTS_REAR_LEFT_LOWER = 5 };
      enum { LIGHTS_REAR_RIGHT_UPPER = 6 };
      enum { LIGHTS_REAR_RIGHT_LOWER = 7 };

    Lights():
      lights()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 8; i++){
      offset += this->lights[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 8; i++){
      offset += this->lights[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    const char * getType(){ return "ridgeback_msgs/Lights"; };
    const char * getMD5(){ return "2c68505ba4cf8e160d2760ed01777bc7"; };

  };

}
#endif