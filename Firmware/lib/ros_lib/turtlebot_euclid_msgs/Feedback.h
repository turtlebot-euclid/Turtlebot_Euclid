#ifndef _ROS_turtlebot_euclid_msgs_Feedback_h
#define _ROS_turtlebot_euclid_msgs_Feedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "turtlebot_euclid_msgs/DriveFeedback.h"

namespace turtlebot_euclid_msgs
{

  class Feedback : public ros::Msg
  {
    public:
      std_msgs::Header header;
      turtlebot_euclid_msgs::DriveFeedback drivers[2];
      enum { LEFT = 0 };
      enum { RIGHT = 1 };

    Feedback():
      header(),
      drivers()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint8_t i = 0; i < 2; i++){
      offset += this->drivers[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint8_t i = 0; i < 2; i++){
      offset += this->drivers[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    const char * getType(){ return "turtlebot_euclid_msgs/Feedback"; };
    const char * getMD5(){ return "a9c8cc740e914a2dd7d1153fdea3e889"; };

  };

}
#endif