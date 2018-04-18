/**
 *
 *  \file       create2_base.h
 *  \brief      Library to interface with the iRobot Create 2.
 *  \author     Tony Baltovski <tbaltovski@clearpathrobotics.com>
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

#ifndef CREATE2_BASE_CREATE2_BASE_H
#define CREATE2_BASE_CREATE2_BASE_H

#include "create2_datatypes.h"
#include "RingBufCPP.h"

#include <SoftwareSerial.h>


class Create2Base
{
private:
  SoftwareSerial* io_;

  uint8_t state_;
  uint32_t last_state_millis_;
  uint8_t desired_oi_mode_;
  uint8_t oi_mode_;
  uint8_t button_leds_[3];

  typedef struct
  {
    bool state;
    bool previous_reading;
    uint32_t last_reading_time;  // [millis]
  }
  ButtonData;

  uint8_t buttons_;
  ButtonData button_data_[8];

  uint8_t bumps_and_wheel_drops_;
  bool bumps_and_wheel_drops_data_[4];

  uint8_t light_bumpers_;
  bool light_bumpers_data_[6];

  bool cliff_data_[4];

  uint8_t charging_state_;
  int16_t battery_current_; // [mA]
  int16_t main_brush_current_;  // [mA]
  uint16_t battery_voltage_;  // [mV]
  uint16_t battery_capacity_; // [mAh]
  uint16_t battery_charge_; // [mAh]
  float battery_;  // [%]
  bool power_main_;

  const uint8_t requested_stream_packets_[16];
  RingBufCPP<uint8_t, 128> rx_buffer_;
  uint8_t stream_buffer_[42];
  uint32_t last_stream_millis_;

  typedef struct
  {
    volatile uint16_t current_ticks;  // [ticks]
    volatile uint16_t previous_ticks;  // [ticks]
    volatile int32_t delta_ticks;  // [ticks]
    volatile float current_velocity;  // [rad/sec]
    volatile float current_travel;  // [radians]
    volatile uint32_t previous_time; // [millis]

    union
    {
      volatile int16_t data;
      volatile uint8_t byte[2];
    } command_velocity;  // [mm/sec]
  }
  EncoderData;

  EncoderData encoders_[2];

public:
  static const uint8_t LOW_BYTE = 0;
  static const uint8_t HIGH_BYTE = 1;
  static const uint8_t START_BYTE = 19;
  static const uint8_t EXPECTED_BYTES = 39;
  static const uint8_t TOTAL_BYTES = 42;
  static const uint8_t REQUESTED_BYTES = 16;

  static const int16_t MAX_SPEED = 500; // [mm/s]
  static constexpr float WHEEL_RADIUS = 36; // [mm]
  static constexpr float COUNTS_PER_REV = 508.8; // [counts/rev]

  Create2Base(SoftwareSerial* io);

  void setup();

  void startStream();
  void getDataStream();
  bool readDataStream();
  void updateData();
  void updateState(uint32_t spin_millis);
  bool hasNewData();
  void spinOnce();

  inline void setBit(uint8_t& variable, uint8_t number);
  inline void clearBit(uint8_t& variable, uint8_t number);
  inline void toggleBit(uint8_t& variable, uint8_t number);
  inline bool extractBit(const uint8_t& variable, uint8_t position);
  inline int16_t bound(const int16_t& value, int16_t min, int16_t max);
  inline int8_t getSign(const uint8_t& variable);

  void setOIMode(uint8_t mode);
  void setPowerLed(uint8_t colour, uint8_t intensity);
  void setDebrisLed(bool enable);
  void setSpotLed(bool enable);
  void setDockLed(bool enable);
  void setWarningLed(bool enable);
  void setBrushMotor(bool enable);
  void updateLEDs();
  void writeLedDisplay(uint8_t& a, uint8_t& b, uint8_t& c, uint8_t& d);

  bool getMainPower();
  uint8_t getOIMode();
  uint8_t getChargingState();
  float getBatteryVoltage();
  float getBatteryCharge();
  float getBatteryCurrent();
  float getBatteryCapacity();
  float getMainBurshCurrent();
  bool getButtonPress(uint8_t button);
  bool getWheelDrop(uint8_t side);
  bool getBumpDrop(uint8_t side);
  bool getCliff(uint8_t location);
  bool getLightBump(uint8_t location);

  void dock();
  void undock();
  void debounce(ButtonData& data, uint32_t& reading_time, bool reading);
  float getTravel(uint8_t side);
  float getVelocity(uint8_t side);
  void sendCmdVel(float left_vel, float right_vel);
  void updateEncoder(EncoderData& encoder, uint32_t& current_time);
};

enum
{
  LEFT = 0,
  RIGHT = 1
};


namespace States
{
  enum State
  {
    PreReset,
    Reset,
    ResetDelay,
    Prestart,
    Start,
    StartDelay,
    Stream,
    StreamDelay,
    Running,
    RunningDelay
  };
}
typedef States::State State;

#endif // CREATE2_BASE_CREATE2_BASE_H
