#ifndef _ROS_turtlebot_euclid_msgs_Status_h
#define _ROS_turtlebot_euclid_msgs_Status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/duration.h"

namespace turtlebot_euclid_msgs
{

  class Status : public ros::Msg
  {
    public:
      std_msgs::Header header;
      ros::Duration mcu_uptime;
      ros::Duration connection_uptime;
      const char* hardware_id;
      uint8_t oi_mode;
      uint8_t charging_state;
      float battery_voltage;
      float battery_charge;
      float battery_capacity;
      float battery_current;
      float main_brush_current;
      bool button_press[8];
      bool wheel_drop[2];
      bool bump[2];
      bool cliff[4];
      bool light_bump[6];
      enum { OI_MODE_OFF =  0 };
      enum { OI_MODE_PASSIVE =  1 };
      enum { OI_MODE_SAFE =  2 };
      enum { OI_MODE_FULL =  3 };
      enum { CHARGING_STATE_NOT_CHARGING =  0 };
      enum { CHARGING_STATE_RECONDITIONING_CHARGING =  1 };
      enum { CHARGING_STATE_FULLY_CHARGING =  2 };
      enum { CHARGING_STATE_TRICKLE_CHARGING =  3 };
      enum { CHARGING_STATE_WAITING =  4 };
      enum { CHARGING_STATE_FAULT =  5 };
      enum { CHARGING_STATE_UNKNOWN =  6 };
      enum { BUTTON_CLEAN_PRESSED =  0 };
      enum { BUTTON_SPOT_PRESSED =  1 };
      enum { BUTTON_DOCK_PRESSED =  2 };
      enum { BUTTON_MINUTE_PRESSED =  3 };
      enum { BUTTON_HOUR_PRESSED =  4 };
      enum { BUTTON_DAY_PRESSED =  5 };
      enum { BUTTON_SCHEDULE_PRESSED =  6 };
      enum { BUTTON_CLOCK_PRESSED =  7 };
      enum { LEFT =  0 };
      enum { RIGHT =  1 };
      enum { CLIFF_LEFT =  0 };
      enum { CLIFF_FRONT_LEFT =  1 };
      enum { CLIFF_FRONT_RIGHT =  2 };
      enum { CLIFF_RIGHT =  3 };
      enum { LIGHT_BUMP_LEFT =  0 };
      enum { LIGHT_BUMP_FRONT_LEFT =  1 };
      enum { LIGHT_BUMP_CENTER_LEFT =  2 };
      enum { LIGHT_BUMP_CENTER_RIGHT =  3 };
      enum { LIGHT_BUMP_FRONT_RIGHT =  4 };
      enum { LIGHT_BUMP_RIGHT =  5 };

    Status():
      header(),
      mcu_uptime(),
      connection_uptime(),
      hardware_id(""),
      oi_mode(0),
      charging_state(0),
      battery_voltage(0),
      battery_charge(0),
      battery_capacity(0),
      battery_current(0),
      main_brush_current(0),
      button_press(),
      wheel_drop(),
      bump(),
      cliff(),
      light_bump()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->mcu_uptime.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mcu_uptime.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mcu_uptime.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mcu_uptime.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mcu_uptime.sec);
      *(outbuffer + offset + 0) = (this->mcu_uptime.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mcu_uptime.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mcu_uptime.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mcu_uptime.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mcu_uptime.nsec);
      *(outbuffer + offset + 0) = (this->connection_uptime.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->connection_uptime.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->connection_uptime.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->connection_uptime.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->connection_uptime.sec);
      *(outbuffer + offset + 0) = (this->connection_uptime.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->connection_uptime.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->connection_uptime.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->connection_uptime.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->connection_uptime.nsec);
      uint32_t length_hardware_id = strlen(this->hardware_id);
      memcpy(outbuffer + offset, &length_hardware_id, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->hardware_id, length_hardware_id);
      offset += length_hardware_id;
      *(outbuffer + offset + 0) = (this->oi_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->oi_mode);
      *(outbuffer + offset + 0) = (this->charging_state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->charging_state);
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.real = this->battery_voltage;
      *(outbuffer + offset + 0) = (u_battery_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_voltage);
      union {
        float real;
        uint32_t base;
      } u_battery_charge;
      u_battery_charge.real = this->battery_charge;
      *(outbuffer + offset + 0) = (u_battery_charge.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_charge.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_charge.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_charge.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_charge);
      union {
        float real;
        uint32_t base;
      } u_battery_capacity;
      u_battery_capacity.real = this->battery_capacity;
      *(outbuffer + offset + 0) = (u_battery_capacity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_capacity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_capacity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_capacity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_capacity);
      union {
        float real;
        uint32_t base;
      } u_battery_current;
      u_battery_current.real = this->battery_current;
      *(outbuffer + offset + 0) = (u_battery_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_current);
      union {
        float real;
        uint32_t base;
      } u_main_brush_current;
      u_main_brush_current.real = this->main_brush_current;
      *(outbuffer + offset + 0) = (u_main_brush_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_main_brush_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_main_brush_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_main_brush_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->main_brush_current);
      for( uint8_t i = 0; i < 8; i++){
      union {
        bool real;
        uint8_t base;
      } u_button_pressi;
      u_button_pressi.real = this->button_press[i];
      *(outbuffer + offset + 0) = (u_button_pressi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->button_press[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      union {
        bool real;
        uint8_t base;
      } u_wheel_dropi;
      u_wheel_dropi.real = this->wheel_drop[i];
      *(outbuffer + offset + 0) = (u_wheel_dropi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->wheel_drop[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      union {
        bool real;
        uint8_t base;
      } u_bumpi;
      u_bumpi.real = this->bump[i];
      *(outbuffer + offset + 0) = (u_bumpi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bump[i]);
      }
      for( uint8_t i = 0; i < 4; i++){
      union {
        bool real;
        uint8_t base;
      } u_cliffi;
      u_cliffi.real = this->cliff[i];
      *(outbuffer + offset + 0) = (u_cliffi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cliff[i]);
      }
      for( uint8_t i = 0; i < 6; i++){
      union {
        bool real;
        uint8_t base;
      } u_light_bumpi;
      u_light_bumpi.real = this->light_bump[i];
      *(outbuffer + offset + 0) = (u_light_bumpi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->light_bump[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->mcu_uptime.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->mcu_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mcu_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->mcu_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->mcu_uptime.sec);
      this->mcu_uptime.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->mcu_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mcu_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->mcu_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->mcu_uptime.nsec);
      this->connection_uptime.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->connection_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->connection_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->connection_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->connection_uptime.sec);
      this->connection_uptime.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->connection_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->connection_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->connection_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->connection_uptime.nsec);
      uint32_t length_hardware_id;
      memcpy(&length_hardware_id, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_hardware_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_hardware_id-1]=0;
      this->hardware_id = (char *)(inbuffer + offset-1);
      offset += length_hardware_id;
      this->oi_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->oi_mode);
      this->charging_state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->charging_state);
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.base = 0;
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_voltage = u_battery_voltage.real;
      offset += sizeof(this->battery_voltage);
      union {
        float real;
        uint32_t base;
      } u_battery_charge;
      u_battery_charge.base = 0;
      u_battery_charge.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_charge.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_charge.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_charge.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_charge = u_battery_charge.real;
      offset += sizeof(this->battery_charge);
      union {
        float real;
        uint32_t base;
      } u_battery_capacity;
      u_battery_capacity.base = 0;
      u_battery_capacity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_capacity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_capacity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_capacity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_capacity = u_battery_capacity.real;
      offset += sizeof(this->battery_capacity);
      union {
        float real;
        uint32_t base;
      } u_battery_current;
      u_battery_current.base = 0;
      u_battery_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_current = u_battery_current.real;
      offset += sizeof(this->battery_current);
      union {
        float real;
        uint32_t base;
      } u_main_brush_current;
      u_main_brush_current.base = 0;
      u_main_brush_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_main_brush_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_main_brush_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_main_brush_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->main_brush_current = u_main_brush_current.real;
      offset += sizeof(this->main_brush_current);
      for( uint8_t i = 0; i < 8; i++){
      union {
        bool real;
        uint8_t base;
      } u_button_pressi;
      u_button_pressi.base = 0;
      u_button_pressi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->button_press[i] = u_button_pressi.real;
      offset += sizeof(this->button_press[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      union {
        bool real;
        uint8_t base;
      } u_wheel_dropi;
      u_wheel_dropi.base = 0;
      u_wheel_dropi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->wheel_drop[i] = u_wheel_dropi.real;
      offset += sizeof(this->wheel_drop[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      union {
        bool real;
        uint8_t base;
      } u_bumpi;
      u_bumpi.base = 0;
      u_bumpi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->bump[i] = u_bumpi.real;
      offset += sizeof(this->bump[i]);
      }
      for( uint8_t i = 0; i < 4; i++){
      union {
        bool real;
        uint8_t base;
      } u_cliffi;
      u_cliffi.base = 0;
      u_cliffi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cliff[i] = u_cliffi.real;
      offset += sizeof(this->cliff[i]);
      }
      for( uint8_t i = 0; i < 6; i++){
      union {
        bool real;
        uint8_t base;
      } u_light_bumpi;
      u_light_bumpi.base = 0;
      u_light_bumpi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->light_bump[i] = u_light_bumpi.real;
      offset += sizeof(this->light_bump[i]);
      }
     return offset;
    }

    const char * getType(){ return "turtlebot_euclid_msgs/Status"; };
    const char * getMD5(){ return "797e840bbf86861bd2cf2c60ad8857b8"; };

  };

}
#endif