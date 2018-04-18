#ifndef _ROS_turtlebot_euclid_msgs_DriveFeedback_h
#define _ROS_turtlebot_euclid_msgs_DriveFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace turtlebot_euclid_msgs
{

  class DriveFeedback : public ros::Msg
  {
    public:
      float measured_travel;
      float measured_velocity;

    DriveFeedback():
      measured_travel(0),
      measured_velocity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_measured_travel;
      u_measured_travel.real = this->measured_travel;
      *(outbuffer + offset + 0) = (u_measured_travel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_measured_travel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_measured_travel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_measured_travel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->measured_travel);
      union {
        float real;
        uint32_t base;
      } u_measured_velocity;
      u_measured_velocity.real = this->measured_velocity;
      *(outbuffer + offset + 0) = (u_measured_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_measured_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_measured_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_measured_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->measured_velocity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_measured_travel;
      u_measured_travel.base = 0;
      u_measured_travel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_measured_travel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_measured_travel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_measured_travel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->measured_travel = u_measured_travel.real;
      offset += sizeof(this->measured_travel);
      union {
        float real;
        uint32_t base;
      } u_measured_velocity;
      u_measured_velocity.base = 0;
      u_measured_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_measured_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_measured_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_measured_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->measured_velocity = u_measured_velocity.real;
      offset += sizeof(this->measured_velocity);
     return offset;
    }

    const char * getType(){ return "turtlebot_euclid_msgs/DriveFeedback"; };
    const char * getMD5(){ return "735e5cc389becbac2b4ec75e3add9490"; };

  };

}
#endif