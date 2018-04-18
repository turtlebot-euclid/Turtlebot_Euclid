/**
 *
 *  \file       create2_datatypes.h
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

#ifndef CREATE2_BASE_CREATE2_DATATYPES_H
#define CREATE2_BASE_CREATE2_DATATYPES_H

namespace ButtonLeds
{
  enum ButtonLed
  {
    Bits = 0,
    PowerColour = 1,
    PowerIntensity = 2,
    Green = 0,
    Red = 255,
    Off = 0,
    Full = 255
  };
}
typedef ButtonLeds::ButtonLed ButtonLed;

namespace SensorPackets
{
  enum SensorPacket
  {
    BumpsAndWheelDrops = 7,  // uint8, bit booleans
    Wall = 8, // uint8, bool
    CliffLeft = 9, // uint8, bool
    CliffFrontLeft = 10, // uint8, bool
    CliffFrontRight = 11, // uint8, bool
    CliffRight = 12, // uint8, bool
    VirtualWall = 13, // uint8, bool
    WheelOvercurrents = 14,  // uint8, bit booleans
    DirtDetect = 15, // uint8
    UnusedByte = 16, // uint8
    InfraredCharacterOmni = 17, // uint8
    InfraredCharacterLeft = 52, // uint8
    InfraredCharacterRight = 53, // uint8
    Buttons = 18, // uint8,  bit booleans
    Distance = 19, // int16 [mm]
    Angle = 20, // int16 [degrees]
    ChargingState = 21, // uint8
    Voltage = 22, // uint16 [mV]
    Current = 23, // int16 [mA]
    Temperature = 24, // int8 [C]
    BatteryCharge = 25, // uint16 [mAh]
    BatteryCapacity = 26, // uint16 [mAh]
    WallSignal = 27, // uint16
    CliffLeftSignal = 28, // uint16
    CliffFrontLeftSignal = 29, // uint16
    CliffFrontRightSignal = 30, // uint16
    CliffRightSignal = 31, // uint16
    ChargingSourcesAvailable = 34, // uint8
    OIMode = 35, // uint8
    SongNumber = 36, // uint8
    SongPlaying = 37, // uint8
    NumberOfStreamPackets = 38, // uint8
    RequestedVelocity = 39, // int16 [mm/s]
    RequestedRadius  = 40, // int16 [mm]
    RequestedRightVelocity  = 41, // int16 [mm]
    RequestedLeftVelocity  = 42, // int16 [mm]
    LeftEncoderCounts  = 43, // uint16
    RightEncoderCounts = 44, // uint16
    LightBumper = 45, // uint8
    LightBumpLeftSignal = 46, // uint16
    LightBumpFrontLeftSignal = 47, // uint16
    LightBumpCenterLeftSignal = 48, // uint16
    LightBumpCenterRightSignal = 49, // uint16
    LightBumpFrontRightSignal = 50, // uint16
    LightBumpRightSignal = 51, // uint16
    LeftMotorCurrent  = 54, // int16 [mA]
    RightMotorCurrent = 55, // int16 [mA]
    MainBrushMotorCurrent  = 56, // int16 [mA]
    SideBrushCurrent = 57,  // int16 [mA]
    Statis  = 58, // uint8
  };
}
typedef SensorPackets::SensorPacket SensorPacket;

namespace Opcodes
{
enum Opcode
{
  Reset = 7,
  Start = 128,
  Buad = 129,
  Control = 130,
  Safe = 131,
  Full = 132,
  Power = 133,
  Spot = 134,
  Clean = 135,
  MaxClean = 136,
  Drive = 137,
  Motors = 138,
  Leds = 139,
  Song = 140,
  Play = 141,
  Query = 142,
  ForceSeekingDock = 143,
  PwmMotors = 144,
  DriveWheels = 145,
  DrivePwm = 146,
  Stream = 148,
  QueryList = 149,
  DoStream = 150,
  SchedulingLeds = 162,
  DigitLedsRaw = 163,
  DigitLedsAscii = 164,
  Buttons = 165,
  Sechedule = 167,
  SetDataAndTime = 168,
  Stop = 173
};
}

typedef Opcodes::Opcode Opcode;


// Modes available for the Create2
namespace Modes
{
enum Mode
{
  Off = 0,
  Passive = 1,
  Safe = 2,
  Full = 3,
  Unknown = 4
};
}
typedef Modes::Mode Mode;


namespace ChargeStates
{
enum ChargeState
{
  NotCharging = 0,
  ReconditioningCharging = 1,
  FullCharging = 2,
  TrickleCharging = 3,
  Waiting = 4,
  Fault = 5,
  Unknown = 6
};
}
typedef ChargeStates::ChargeState ChargeState;

namespace BaudRates
{
enum BaudRate
{
  B300 = 0,
  B600 = 1,
  B1200 = 2,
  B2400 = 3,
  B4800 = 4,
  B9600 = 5,
  B14400 = 6,
  B19200 = 7,
  B28800 = 8,
  B38400 = 9,
  B57600 = 10,
  B115200 = 11
};
}

typedef BaudRates::BaudRate BaudRate;


namespace ButtonMasks
{
enum ButtonMask
{
  Clean = 0,
  Spot = 1,
  Dock = 2,
  Minute = 3,
  Hour = 4,
  Day = 5,
  Schedule = 6,
  Clock = 7
};
}

typedef ButtonMasks::ButtonMask ButtonMask;


namespace LightBumperMasks
{
enum LightBumperMask
{
  Left = 0,
  FrontLeft = 1,
  CenterLeft = 2,
  CenterRight = 3,
  FrontRight = 4,
  Right = 5
};
}

typedef LightBumperMasks::LightBumperMask LightBumperMask;

namespace BumpsAndWheelDropsMasks
{
enum BumpsAndWheelDropsMask
{
  BumpRight = 0,
  BumpLeft = 1,
  WheelDropRight = 2,
  WheelDropLeft = 3
};
}

typedef BumpsAndWheelDropsMasks::BumpsAndWheelDropsMask BumpsAndWheelDropsMask;

namespace ChargerAvailableMasks
{
enum ChargerAvailableMask
{
  HomeBase = 0,
  InternalCharger = 1
};
}

typedef ChargerAvailableMasks::ChargerAvailableMask ChargerAvailableMask;


namespace OvercurrentsMasks
{
enum OvercurrentsMask
{
  SideBrush = 0,
  MainBrush = 2,
  RightWheel = 3,
  LeftWheel = 4
};
}

typedef OvercurrentsMasks::OvercurrentsMask OvercurrentsMask;


namespace LightsMasks
{
enum LightsMask
{
  Debris = 0,
  Spot = 1,
  Dock = 2,
  Warning = 3
};
}

typedef LightsMasks::LightsMask LightsMask;

namespace CliffMasks
{
enum CliffMask
{
  Left = 0,
  FrontLeft = 1,
  FrontRight = 2,
  Right = 3
};
}

typedef CliffMasks::CliffMask CliffMask;

#endif  // CREATE2_BASE_CREATE2_DATATYPES_H
