/**
 *
 *  \file
 *  \brief      Firmware for Turtlebot Euclid.
 *  \author     Tony Baltovski <tbaltovski@clearpathrobotics.com>
 *  \author     Tom Sun <tsun@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2017, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

#include "create2_base.h"
#include "hardware.h"
#include "firmware_commit.h"

#include <ros.h>
#include <ros/time.h>
#include <ros/duration.h>

#include <turtlebot_euclid_msgs/Command.h>
#include <turtlebot_euclid_msgs/Drive.h>
#include <turtlebot_euclid_msgs/Feedback.h>
#include <turtlebot_euclid_msgs/Status.h>

extern const char* g_firmware_commit;

SoftwareSerial create2(MCU::GPIO::RX_SW, MCU::GPIO::TX_SW, false, 256);
Create2Base robot_base(&create2);

ros::NodeHandle_<ArduinoHardware, 2, 2, 1024, 1024> nh;

turtlebot_euclid_msgs::Feedback feedback_msg;
ros::Publisher pub_feedback("turtlebot_euclid/feedback", &feedback_msg);

turtlebot_euclid_msgs::Status status_msg;
ros::Publisher pub_status("turtlebot_euclid/status", &status_msg);

uint32_t last_cmd_vel_millis = 0;
uint32_t last_rosserial_millis = 0;

// calculated encoder ticks, velocity and distance from streamed data
void publishFeedback()
{
  feedback_msg.header.stamp = nh.now();
  feedback_msg.drivers[LEFT].measured_travel = robot_base.getTravel(LEFT);
  feedback_msg.drivers[RIGHT].measured_travel = robot_base.getTravel(RIGHT);
  feedback_msg.drivers[LEFT].measured_velocity = robot_base.getVelocity(LEFT);
  feedback_msg.drivers[RIGHT].measured_velocity = robot_base.getVelocity(RIGHT);
  pub_feedback.publish(&feedback_msg);
}

void publishStatus()
{
  status_msg.header.stamp = nh.now();
  status_msg.mcu_uptime = ros::Duration(millis() / 1000, 0);
  status_msg.connection_uptime = ros::Duration((millis() - last_rosserial_millis) / 1000, 0);
  status_msg.hardware_id = g_firmware_commit;
  status_msg.oi_mode = robot_base.getOIMode();
  status_msg.charging_state = robot_base.getChargingState();
  status_msg.battery_voltage = robot_base.getBatteryVoltage();
  status_msg.battery_charge = robot_base.getBatteryCharge();
  status_msg.battery_capacity = robot_base.getBatteryCapacity();
  status_msg.battery_current = robot_base.getBatteryCurrent();
  status_msg.main_brush_current = robot_base.getMainBurshCurrent();
  for (uint8_t i = 0; i < 8; i++)
  {
    status_msg.button_press[i] = robot_base.getButtonPress(i);
  }
  for (uint8_t i = 0; i < 2; i++)
  {
    status_msg.wheel_drop[i] = robot_base.getWheelDrop(i);
    status_msg.bump[i] = robot_base.getBumpDrop(i);
  }
  for (uint8_t i = 0; i < 4; i++)
  {
    status_msg.cliff[i] = robot_base.getCliff(i);
  }
  for (uint8_t i = 0; i < 6; i++)
  {
    status_msg.light_bump[i] = robot_base.getLightBump(i);
  }
  pub_status.publish(&status_msg);
}

// call back for turtlebot node (cmd_vel)
void driveCallback(const turtlebot_euclid_msgs::Drive& drive_msg)
{
  robot_base.sendCmdVel(drive_msg.drivers[turtlebot_euclid_msgs::Drive::LEFT],
                        drive_msg.drivers[turtlebot_euclid_msgs::Drive::RIGHT]);
  last_cmd_vel_millis = millis();
}

void cmdCallback(const turtlebot_euclid_msgs::Command& cmd_msg)
{
  if (cmd_msg.oi_mode_change)
  {
    robot_base.setOIMode(cmd_msg.oi_mode);
  }

  if (cmd_msg.seek_dock)
  {
    robot_base.dock();
  }

  robot_base.setDockLed(cmd_msg.enable_debris_led);

}

// topic /cmd_drive gives an array for right and left wheel velocities
ros::Subscriber<turtlebot_euclid_msgs::Drive> sub_drive("turtlebot_euclid/cmd_drive", &driveCallback);
ros::Subscriber<turtlebot_euclid_msgs::Command> sub_cmd("turtlebot_euclid/cmd_base", &cmdCallback);

/*************************************************************
  SETUP
*************************************************************/
void setup()
{
  // Disable the built-in WiFi, it is not used.
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);

  // Configure the GPIO
  pinMode(MCU::GPIO::LED, OUTPUT);
  pinMode(MCU::GPIO::ENABLE_5V_SUPPLY, OUTPUT);
  pinMode(MCU::GPIO::ENABLE_12V_SUPPLY, OUTPUT);

  //Serial.begin(115200);

  // Configure Create2 base.
  robot_base.setup();

  // Configure rosserial.
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub_drive);
  nh.subscribe(sub_cmd);
  nh.advertise(pub_feedback);
  nh.advertise(pub_status);
}

/*************************************************************
  LOOP
*************************************************************/
void loop()
{
  static uint32_t last_feedback_millis = 0;
  static uint32_t last_status_millis = 0;
  static uint32_t last_blink_millis = 0;
  static bool led_state = false;

  if (!nh.connected())
  {
    if ((millis() - last_blink_millis) > 100)
    {
      digitalWrite(MCU::GPIO::LED, (led_state ^= 1));
      robot_base.setSpotLed(led_state);
      digitalWrite(MCU::GPIO::ENABLE_5V_SUPPLY, robot_base.getMainPower());
      digitalWrite(MCU::GPIO::ENABLE_12V_SUPPLY, robot_base.getMainPower());
      last_blink_millis = millis();
      last_rosserial_millis = last_blink_millis;
    }
  }
  else
  {
    if ((millis() - last_blink_millis) > 1000)
    {
      last_blink_millis = millis();
      digitalWrite(MCU::GPIO::LED, (led_state ^= 1));
      robot_base.setSpotLed(!led_state);
      digitalWrite(MCU::GPIO::ENABLE_5V_SUPPLY, robot_base.getMainPower());
      digitalWrite(MCU::GPIO::ENABLE_12V_SUPPLY, robot_base.getMainPower());
    }
  }

  robot_base.spinOnce();
  if (robot_base.hasNewData())
  {
    if ((millis() - last_feedback_millis) > 100)
    {
      last_feedback_millis = millis();
      if (nh.connected())
      {
        publishFeedback();
      }
    }
    if ((millis() - last_status_millis) > 200)
    {
      last_status_millis = millis();
      if (nh.connected())
      {
        publishStatus();
      }
    }
  }

  if ((millis() - last_cmd_vel_millis) > 200)
  {
    robot_base.sendCmdVel(0, 0);
    last_cmd_vel_millis = millis();
  }

  nh.spinOnce();
}
