#ifndef _ROS_turtlebot_euclid_msgs_Command_h
#define _ROS_turtlebot_euclid_msgs_Command_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace turtlebot_euclid_msgs
{

  class Command : public ros::Msg
  {
    public:
      bool oi_mode_change;
      uint8_t oi_mode;
      bool seek_dock;
      bool enable_debris_led;
      uint8_t led_disp[4];
      enum { OI_MODE_OFF =  0 };
      enum { OI_MODE_PASSIVE =  1 };
      enum { OI_MODE_SAFE =  2 };
      enum { OI_MODE_FULL =  3 };

    Command():
      oi_mode_change(0),
      oi_mode(0),
      seek_dock(0),
      enable_debris_led(0),
      led_disp()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_oi_mode_change;
      u_oi_mode_change.real = this->oi_mode_change;
      *(outbuffer + offset + 0) = (u_oi_mode_change.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->oi_mode_change);
      *(outbuffer + offset + 0) = (this->oi_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->oi_mode);
      union {
        bool real;
        uint8_t base;
      } u_seek_dock;
      u_seek_dock.real = this->seek_dock;
      *(outbuffer + offset + 0) = (u_seek_dock.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->seek_dock);
      union {
        bool real;
        uint8_t base;
      } u_enable_debris_led;
      u_enable_debris_led.real = this->enable_debris_led;
      *(outbuffer + offset + 0) = (u_enable_debris_led.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable_debris_led);
      for( uint8_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->led_disp[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->led_disp[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_oi_mode_change;
      u_oi_mode_change.base = 0;
      u_oi_mode_change.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->oi_mode_change = u_oi_mode_change.real;
      offset += sizeof(this->oi_mode_change);
      this->oi_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->oi_mode);
      union {
        bool real;
        uint8_t base;
      } u_seek_dock;
      u_seek_dock.base = 0;
      u_seek_dock.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->seek_dock = u_seek_dock.real;
      offset += sizeof(this->seek_dock);
      union {
        bool real;
        uint8_t base;
      } u_enable_debris_led;
      u_enable_debris_led.base = 0;
      u_enable_debris_led.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable_debris_led = u_enable_debris_led.real;
      offset += sizeof(this->enable_debris_led);
      for( uint8_t i = 0; i < 4; i++){
      this->led_disp[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->led_disp[i]);
      }
     return offset;
    }

    const char * getType(){ return "turtlebot_euclid_msgs/Command"; };
    const char * getMD5(){ return "4c5118a43f449741c230aa0cd73bce41"; };

  };

}
#endif