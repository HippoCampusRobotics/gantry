// Copyright (C) 2023 Thies Lennart Alff
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA

#pragma once

#include <termios.h>

#include <optional>
#include <string>

namespace gantry {

namespace cmd {
static constexpr char kEnable[] = "EN";
static constexpr char kDisable[] = "DI";
static constexpr char kGetType[] = "GTYP";
static constexpr char kGetSerial[] = "GSER";
static constexpr char kGetIOConfig[] = "IOC";

static constexpr char kMove[] = "M";
static constexpr char kStartHoming[] = "GOHOSEQ";
static constexpr char kSetHome[] = "HO";

static constexpr char kSetAbsolutePositionTarget[] = "LA";
static constexpr char kSetRelativePositionTarget[] = "LR";

static constexpr char kGetPosition[] = "POS";
static constexpr char kSetVelocity[] = "V";
static constexpr char kGetVelocity[] = "GN";
static constexpr char kGetVelocityTarget[] = "GV";

static constexpr char kSetVelocityLimit[] = "SP";
static constexpr char kGetVelocityLimit[] = "GSP";
static constexpr char kSetAccelLimit[] = "AC";
static constexpr char kGetAccelLimit[] = "GAC";
static constexpr char kSetDecelLimit[] = "DEC";
static constexpr char kGetDecelLimit[] = "GDEC";
}  // namespace cmd

class Motor {
 public:
  Motor();

  bool InitSerial(std::string _device_name);
  bool SendCommand(const std::string &command);

  inline bool SendCommand(const std::string &command, int arg) {
    return SendCommand(command, std::to_string(arg));
  }

  inline bool SendCommand(const std::string &command, const std::string &arg) {
    return SendCommand(command + arg);
  }

  std::optional<std::string> ReadAnswer();
  std::optional<int> GetInt(const std::string &command);

  bool Enable();
  bool Disable();
  std::optional<std::string> GetType();
  std::optional<std::string> GetSerial();
  /// @brief Current position in motor increments.
  std::optional<int> GetPosition();
  /// @brief Set the absolute position target in motor increments.
  bool SetAbsolutePositionTarget(int position);
  /// @brief Set the position target relative to the current one in motor
  /// increments.
  bool SetRelativePositionTarget(int position);

  bool SetVelocity(int velocity);
  std::optional<int> GetVelocity();
  std::optional<int> GetVelocityTarget();

  bool MoveToCurrentTargetPosition();

  bool SetVelocityLimit(int value);
  std::optional<int> GetVelocityLimit();
  bool SetAccelerationLimit(int value);
  std::optional<int> GetAccelerationLimit();
  bool SetDecelerationLimit(int value);
  std::optional<int> GetDecelerationLimit();

  bool StartHoming();
  virtual std::optional<bool> IsHoming() = 0;
  virtual std::optional<bool> GetLowerLimitSwitch() = 0;
  virtual std::optional<bool> GetUpperLimitSwitch() = 0;
  virtual std::optional<bool> IsEnabled() = 0;
  virtual std::optional<bool> IsPositionReached() = 0;

  /**
   * Set the the home position so that the current position equals `position`.
   * @param position The position to be set in increments.
   */
  bool SetHome(int position);

  int port_;
  std::string device_name_;
  struct termios tty_;
  bool initialized_{false};
};
}  // namespace gantry
