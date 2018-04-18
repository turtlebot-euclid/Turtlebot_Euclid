/**
 *
 *  \file       create2_base.cpp
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


#include "create2_base.h"

Create2Base::Create2Base(SoftwareSerial* io) :
  io_(io),
  state_(State::Prestart),
  last_state_millis_(0),
  requested_stream_packets_ { SensorPacket::LeftEncoderCounts, // 2 bytes
                              SensorPacket::RightEncoderCounts, // 2 bytes
                              SensorPacket::BumpsAndWheelDrops, // 1 byte
                              SensorPacket::LightBumper, // 1 byte
                              SensorPacket::CliffLeft, // 1 byte
                              SensorPacket::CliffFrontLeft, // 1 byte
                              SensorPacket::CliffFrontRight, // 1 byte
                              SensorPacket::CliffRight, // 1 byte
                              SensorPacket::ChargingState, // 1 byte
                              SensorPacket::Current, // 2 bytes
                              SensorPacket::MainBrushMotorCurrent, // 2 bytes
                              SensorPacket::Voltage, // 2 bytes
                              SensorPacket::BatteryCapacity, // 2 bytes
                              SensorPacket::OIMode, // 1 byte
                              SensorPacket::Buttons, // 1 byte
                              SensorPacket::BatteryCharge}, // 2 bytes
  button_leds_{0, ButtonLed::Red, ButtonLed::Full},
  desired_oi_mode_(Mode::Full),
  oi_mode_(Mode::Unknown),
  buttons_(0),
  bumps_and_wheel_drops_(0),
  light_bumpers_(0),
  charging_state_(ChargeState::Unknown),
  battery_current_(0),
  main_brush_current_(0),
  battery_voltage_(0),
  battery_capacity_(0),
  power_main_(false)
{
}

void Create2Base::setup()
{
  io_->begin(115200);
  io_->flush();
}

inline void Create2Base::setBit(uint8_t& variable, uint8_t number)
{
  variable |= 1 << number;
}

inline void Create2Base::clearBit(uint8_t& variable, uint8_t number)
{
  variable &= ~(1 << number);
}

inline void Create2Base::toggleBit(uint8_t& variable, uint8_t number)
{
  variable ^= 1 << number;
}

inline bool Create2Base::extractBit(const uint8_t& variable, uint8_t position)
{
  return ((variable) & (1 << (position)));
}

inline int16_t Create2Base::bound(const int16_t& value, int16_t min, int16_t max)
{
  return ((value < min) ? min : (value > max) ? max : value);
}

inline int8_t Create2Base::getSign(const uint8_t& variable)
{
  return ((variable > 0) ? 1 : ((variable < 0) ? -1 : 0));
}

bool Create2Base::getMainPower()
{
  return power_main_;
}

void Create2Base::startStream()
{
  io_->write(Opcode::Stream);
  io_->write(REQUESTED_BYTES);
  for (uint8_t i = 0; i < REQUESTED_BYTES; i++)
  {
    io_->write(requested_stream_packets_[i]);
  }
}

void Create2Base::getDataStream()
{
  if (io_->available() > 0)
  {
    rx_buffer_.add(io_->read());
  }
}

bool Create2Base::readDataStream()
{
  memset(stream_buffer_, 0, sizeof(stream_buffer_));
  if (rx_buffer_.isEmpty() || (rx_buffer_.numElements() < TOTAL_BYTES))
  {
    return false;
  }
  uint8_t tmp;
  while (rx_buffer_.pull(&tmp))
  {
    if (tmp == START_BYTE)
    {
      stream_buffer_[0] = START_BYTE;
      rx_buffer_.pull(&tmp);
      if (tmp == EXPECTED_BYTES)
      {
        stream_buffer_[1] = EXPECTED_BYTES;
      }
      else
      {
        return false;
      }
      break;
    }
    else
    {
      return false;
    }
  }
  uint16_t check_sum = START_BYTE + EXPECTED_BYTES;
  for (uint8_t i = 2; i < TOTAL_BYTES; i++)
  {
    rx_buffer_.pull(&tmp);
    stream_buffer_[i] = tmp;
    check_sum += tmp;
  }
  if ((check_sum & 0xFF) != 0)
  {
    return false;
  }
  last_stream_millis_ = millis();
  return true;
}

void Create2Base::updateData()
{
  if (stream_buffer_[2] == SensorPacket::LeftEncoderCounts)
  {
    encoders_[LEFT].current_ticks = static_cast<uint16_t>((stream_buffer_[3] << 8) | stream_buffer_[4]);
    updateEncoder(encoders_[LEFT], last_stream_millis_);
  }

  if (stream_buffer_[5] == SensorPacket::RightEncoderCounts)
  {
    encoders_[RIGHT].current_ticks = static_cast<uint16_t>((stream_buffer_[6] << 8) | stream_buffer_[7]);
    updateEncoder(encoders_[RIGHT], last_stream_millis_);
  }

  if (stream_buffer_[8] == SensorPacket::BumpsAndWheelDrops)
  {
    bumps_and_wheel_drops_ = stream_buffer_[9];
    bumps_and_wheel_drops_data_[BumpsAndWheelDropsMask::BumpLeft]
      = extractBit(bumps_and_wheel_drops_, BumpsAndWheelDropsMask::BumpLeft);
    bumps_and_wheel_drops_data_[BumpsAndWheelDropsMask::BumpRight]
      = extractBit(bumps_and_wheel_drops_, BumpsAndWheelDropsMask::BumpRight);
    bumps_and_wheel_drops_data_[BumpsAndWheelDropsMask::WheelDropLeft]
      = extractBit(bumps_and_wheel_drops_, BumpsAndWheelDropsMask::WheelDropLeft);
    bumps_and_wheel_drops_data_[BumpsAndWheelDropsMask::WheelDropRight]
      = extractBit(bumps_and_wheel_drops_, BumpsAndWheelDropsMask::WheelDropRight);
  }

  if (stream_buffer_[10] == SensorPacket::LightBumper)
  {
    light_bumpers_ = stream_buffer_[11];
    light_bumpers_data_[LightBumperMask::Left] = extractBit(light_bumpers_, LightBumperMask::Left);
    light_bumpers_data_[LightBumperMask::FrontLeft] = extractBit(light_bumpers_, LightBumperMask::FrontLeft);
    light_bumpers_data_[LightBumperMask::CenterLeft] = extractBit(light_bumpers_, LightBumperMask::CenterLeft);
    light_bumpers_data_[LightBumperMask::CenterRight] = extractBit(light_bumpers_, LightBumperMask::CenterRight);
    light_bumpers_data_[LightBumperMask::FrontRight] = extractBit(light_bumpers_, LightBumperMask::FrontRight);
    light_bumpers_data_[LightBumperMask::Right] = extractBit(light_bumpers_, LightBumperMask::Right);
  }

  if (stream_buffer_[12] == SensorPacket::CliffLeft)
  {
    cliff_data_[CliffMask::Left] = stream_buffer_[13];
  }

  if (stream_buffer_[14] == SensorPacket::CliffFrontLeft)
  {
    cliff_data_[CliffMask::FrontLeft] = stream_buffer_[15];
  }

  if (stream_buffer_[16] == SensorPacket::CliffFrontRight)
  {
    cliff_data_[CliffMask::FrontRight] = stream_buffer_[17];
  }

  if (stream_buffer_[18] == SensorPacket::CliffRight)
  {
    cliff_data_[CliffMask::Right] = stream_buffer_[19];
  }

  if (stream_buffer_[20] == SensorPacket::ChargingState)
  {
    charging_state_ = stream_buffer_[21];
  }

  if (stream_buffer_[22] == SensorPacket::Current)
  {
    battery_current_ = static_cast<int16_t>((stream_buffer_[23] << 8) | stream_buffer_[24]);
  }

  if (stream_buffer_[25] == SensorPacket::MainBrushMotorCurrent)
  {
    main_brush_current_ = static_cast<int16_t>((stream_buffer_[26] << 8) | stream_buffer_[27]);
  }

  if (stream_buffer_[28] == SensorPacket::Voltage)
  {
    battery_voltage_ = static_cast<uint16_t>((stream_buffer_[29] << 8) | stream_buffer_[30]);
  }

  if (stream_buffer_[31] == SensorPacket::BatteryCapacity)
  {
    battery_capacity_ = static_cast<uint16_t>((stream_buffer_[32] << 8) | stream_buffer_[33]);
  }

  if (stream_buffer_[34] == SensorPacket::OIMode)
  {
    oi_mode_ = stream_buffer_[35];
  }

  if (stream_buffer_[36] == SensorPacket::Buttons)
  {
    buttons_ = stream_buffer_[37];
    debounce(button_data_[ButtonMask::Day], last_stream_millis_, extractBit(buttons_, ButtonMask::Day));
    debounce(button_data_[ButtonMask::Hour], last_stream_millis_, extractBit(buttons_, ButtonMask::Hour));
    debounce(button_data_[ButtonMask::Minute], last_stream_millis_, extractBit(buttons_, ButtonMask::Minute));
    debounce(button_data_[ButtonMask::Schedule], last_stream_millis_, extractBit(buttons_, ButtonMask::Schedule));
    debounce(button_data_[ButtonMask::Clock], last_stream_millis_, extractBit(buttons_, ButtonMask::Clock));
  }

  if (stream_buffer_[38] == SensorPacket::BatteryCharge)
  {
    battery_charge_ = static_cast<uint16_t>((stream_buffer_[39] << 8) | stream_buffer_[40]);
    battery_ = static_cast<float>(battery_charge_) / static_cast<float>(battery_capacity_);
  }

  if (battery_ < 0.15)
  {
    setWarningLed(true);
    setPowerLed(125, 255);
  }
  // Check if the battery is low to change to passive mode to avoid depleting the battery.
  if (battery_ < 0.10)
  {
    dock();
  }
}

void Create2Base::updateState(uint32_t spin_millis)
{
  // Don't update more than 10Hz.
  if ((spin_millis - last_state_millis_) < 100)
  {
    return;
  }

  if (button_data_[ButtonMask::Day].state && oi_mode_ == Mode::Full)
  {
    power_main_ = true;
    setDockLed(power_main_);
    delay(10);
    setBrushMotor(power_main_);
    delay(1);
  }

  if (button_data_[ButtonMask::Hour].state && oi_mode_ == Mode::Full)
  {
    power_main_ = false;
    setDockLed(power_main_);
    delay(10);
    setBrushMotor(power_main_);
    delay(1);
  }
  if (button_data_[ButtonMask::Minute].state)
  {
    if (desired_oi_mode_ == Mode::Full && oi_mode_ == Mode::Full)
    {
      desired_oi_mode_ = Mode::Passive;
    }
    else if (desired_oi_mode_ == Mode::Passive && oi_mode_ == Mode::Passive)
    {
      desired_oi_mode_ = Mode::Full;
    }
  }

  switch (state_)
  {
    case State::PreReset:
      if ((spin_millis - last_state_millis_) > 5000)
      {
        last_state_millis_ = spin_millis;
        state_ = State::Reset;
      }
      break;
    case State::Reset:
      io_->write(Opcode::Reset);
      state_ = State::ResetDelay;
      last_state_millis_ = spin_millis;
      break;
    case State::ResetDelay:
      if ((spin_millis - last_state_millis_) > 4000)
      {
        last_state_millis_ = spin_millis;
        state_ = State::Prestart;
      }
      break;
    case State::Prestart:
      if ((spin_millis - last_state_millis_) > 1000)
      {
        last_state_millis_ = spin_millis;
        state_ = State::Start;
      }
      break;
    case State::Start:
      io_->write(Opcode::Start);
      state_ = State::StartDelay;
      last_state_millis_ = spin_millis;
      break;
    case State::StartDelay:
      if ((spin_millis - last_state_millis_) > 500)
      {
        last_state_millis_ = spin_millis;
        state_ = State::Stream;
      }
      break;
    case State::Stream:
      startStream();
      state_ = State::StreamDelay;
      last_state_millis_ = spin_millis;
      break;
    case State::StreamDelay:
      if ((spin_millis - last_state_millis_) > 500)
      {
        last_state_millis_ = spin_millis;
        state_ = State::Running;
      }
      break;
    case State::Running:
      if (desired_oi_mode_ != oi_mode_)
      {
        if (oi_mode_ == Mode::Unknown)
        {
          io_->write(Opcode::Start);
          delay(1);
          io_->write(Opcode::Full);
          delay(1);
          setDockLed(power_main_);
          delay(10);
          setBrushMotor(power_main_);
          delay(1);
        }
        else if (desired_oi_mode_ == Mode::Full)
        {
          io_->write(Opcode::Full);
          delay(1);
          setDockLed(power_main_);
          delay(10);
          setBrushMotor(power_main_);
          delay(1);
        }
        else if (desired_oi_mode_ == Mode::Safe)
        {
          io_->write(Opcode::Safe);
          delay(1);
          setDockLed(power_main_);
          delay(10);
          setBrushMotor(power_main_);
          delay(1);
        }
        else if (desired_oi_mode_ == Mode::Passive)
        {
          io_->write(Opcode::Start);
          power_main_ = false;
        }
      }
      last_state_millis_ = spin_millis;
      state_ = State::RunningDelay;
      break;
    case State::RunningDelay:
      if ((spin_millis - last_state_millis_) > 500)
      {
        last_state_millis_ = spin_millis;
        state_ = State::Running;
      }
      break;
  }
}

bool Create2Base::hasNewData()
{

  if (!readDataStream())
  {
    return false;
  }
  updateData();
  return true;
}

void Create2Base::writeLedDisplay(uint8_t& a, uint8_t& b, uint8_t& c, uint8_t& d)
{
  io_->write(Opcode::DigitLedsAscii);
  io_->write(a);
  io_->write(b);
  io_->write(c);
  io_->write(d);
}

void Create2Base::updateLEDs()
{
  io_->write(Opcode::Leds);
  io_->write(button_leds_[ButtonLed::Bits]);
  io_->write(button_leds_[ButtonLed::PowerColour]);
  io_->write(button_leds_[ButtonLed::PowerIntensity]);
}

void Create2Base::setOIMode(uint8_t mode)
{
  desired_oi_mode_ = mode;
}

void Create2Base::setDebrisLed(bool enable)
{
  if (enable)
  {
    setBit(button_leds_[ButtonLed::Bits], LightsMask::Debris);
  }
  else
  {
    clearBit(button_leds_[ButtonLed::Bits], LightsMask::Debris);
  }
  updateLEDs();
}

void Create2Base::setSpotLed(bool enable)
{
  if (enable)
  {
    setBit(button_leds_[ButtonLed::Bits], LightsMask::Spot);
  }
  else
  {
    clearBit(button_leds_[ButtonLed::Bits], LightsMask::Spot);
  }
  updateLEDs();
}

void Create2Base::setDockLed(bool enable)
{
  if (enable)
  {
    setBit(button_leds_[ButtonLed::Bits], LightsMask::Dock);
  }
  else
  {
    clearBit(button_leds_[ButtonLed::Bits], LightsMask::Dock);
  }
  updateLEDs();
}

void Create2Base::setWarningLed(bool enable)
{
  if (enable)
  {
    setBit(button_leds_[ButtonLed::Bits], LightsMask::Warning);
  }
  else
  {
    clearBit(button_leds_[ButtonLed::Bits], LightsMask::Warning);
  }
  updateLEDs();
}

void Create2Base::setPowerLed(uint8_t colour, uint8_t intensity)
{
  button_leds_[ButtonLed::PowerColour] = colour;
  button_leds_[ButtonLed::PowerIntensity] = intensity;
  updateLEDs();
}

void Create2Base::setBrushMotor(bool enable)
{
  io_->write(Opcode::Motors);
  if (enable)
  {
    io_->write(0b00000100);
  }
  else
  {
    io_->write(static_cast<uint8_t>(0));
  }
}

uint8_t Create2Base::getChargingState()
{
  return charging_state_;
}

void Create2Base::dock()
{
  // Set to passive mode.
  io_->write(Opcode::ForceSeekingDock);
  desired_oi_mode_ = Mode::Passive;
  // Disable to the power to the external devices.
  power_main_ = false;
}

void Create2Base::undock()
{
  uint16_t start_time = millis();
  while (millis() - start_time < 3000)
  {
    sendCmdVel(-3.0, -3.0);
  }
  sendCmdVel(0, 0);
}

void Create2Base::debounce(ButtonData& data, uint32_t& reading_time, bool reading)
{
  if (reading != data.previous_reading)
  {
    data.last_reading_time = reading_time;
  }
  else
  {
    if ((reading_time - data.last_reading_time) > 50)
    {
      data.state = reading;
    }
  }
  data.previous_reading = reading;
}

void Create2Base::sendCmdVel(float left_vel, float right_vel)
{
  left_vel *= WHEEL_RADIUS;
  right_vel *= WHEEL_RADIUS;

  encoders_[LEFT].command_velocity.data = bound(static_cast<int16_t>(left_vel), -MAX_SPEED, MAX_SPEED);
  encoders_[RIGHT].command_velocity.data = bound(static_cast<int16_t>(right_vel), -MAX_SPEED, MAX_SPEED);

  io_->write(Opcode::DriveWheels);
  io_->write(encoders_[LEFT].command_velocity.byte[HIGH_BYTE]);
  io_->write(encoders_[LEFT].command_velocity.byte[LOW_BYTE]);
  io_->write(encoders_[RIGHT].command_velocity.byte[HIGH_BYTE]);
  io_->write(encoders_[RIGHT].command_velocity.byte[LOW_BYTE]);
}

void Create2Base::updateEncoder(EncoderData& encoder, uint32_t& current_time)
{
  encoder.delta_ticks = encoder.current_ticks - encoder.previous_ticks;
  if (encoder.delta_ticks < (-UINT16_MAX/2))
  {
    encoder.delta_ticks += UINT16_MAX + 1;  // The one is for jumping through zero.
  }
  else if (encoder.delta_ticks > (UINT16_MAX/2))
  {
    encoder.delta_ticks -= UINT16_MAX - 1;
  }

  encoder.current_travel += (static_cast<float>(encoder.delta_ticks) * 2.0 * M_PI / COUNTS_PER_REV);
  encoder.current_velocity = (((static_cast<float>(encoder.delta_ticks) * 2.0 * M_PI * 1000))
                             / ((current_time - encoder.previous_time) * COUNTS_PER_REV));
  encoder.previous_ticks = encoder.current_ticks;
  encoder.previous_time = current_time;
}

float Create2Base::getTravel(uint8_t side)
{
  return encoders_[side].current_travel;
}

float Create2Base::getVelocity(uint8_t side)
{
  return encoders_[side].current_velocity;
}

uint8_t Create2Base::getOIMode()
{
  return oi_mode_;
}

float Create2Base::getBatteryVoltage()
{
  return static_cast<float>(battery_voltage_) / 1000.0;
}

float Create2Base::getBatteryCharge()
{
  return static_cast<float>(battery_charge_) / 1000.0;
}

float Create2Base::getBatteryCapacity()
{
  return static_cast<float>(battery_capacity_) / 1000.0;
}

float Create2Base::getBatteryCurrent()
{
  return static_cast<float>(battery_current_) / 1000.0;
}

float Create2Base::getMainBurshCurrent()
{
  return static_cast<float>(main_brush_current_) / 1000.0;
}

bool Create2Base::getButtonPress(uint8_t button)
{
  return button_data_[button].state;
}

bool Create2Base::getWheelDrop(uint8_t side)
{
  return bumps_and_wheel_drops_data_[side + 2];
}

bool Create2Base::getBumpDrop(uint8_t side)
{
  return bumps_and_wheel_drops_data_[side];
}

bool Create2Base::getCliff(uint8_t location)
{
  return cliff_data_[location];
}

bool Create2Base::getLightBump(uint8_t location)
{
  return light_bumpers_data_[location];
}


void Create2Base::spinOnce()
{
  getDataStream();
  updateState(millis());
}
