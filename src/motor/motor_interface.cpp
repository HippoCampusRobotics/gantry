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

#include "gantry/motor/motor_interface.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

namespace gantry {

Motor::Motor(){};

bool Motor::InitSerial(std::string _device_name) {
  initialized_ = false;
  device_name_ = _device_name;
  port_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY);
  if (port_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_interface"),
                 "Failed to open serial port '%s'. Exit code: %d",
                 device_name_.c_str(), port_);
    return false;
  }

  cfsetspeed(&tty_, B19200);

  tty_.c_cflag &= ~CRTSCTS;  // disable hardware flow control
  tty_.c_cflag |= CREAD | CLOCAL | CS8;

  tty_.c_iflag |= IGNCR;
  tty_.c_iflag &= ~(IXON | IXOFF | IXANY);

  tty_.c_oflag &= ~(OCRNL | ONLCR);

  tty_.c_lflag &= ~(ECHO | ISIG);
  tty_.c_lflag |= ICANON;

  if (tcsetattr(port_, TCSANOW, &tty_) != 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("motor_interface"),
                        "Could not apply serial port settings!");
    return false;
  }
  tcflush(port_, TCIOFLUSH);
  initialized_ = true;
  return true;
}

bool Motor::SendCommand(const std::string &_command) {
  std::string buffer = _command + "\r";
  int n_written = write(port_, buffer.c_str(), buffer.size());
  // Make sure all data has actually been transmitted. Otherwise data may
  // queue up if we try to send commands faster than they can be transmitted.
  tcdrain(port_);
  if (n_written < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("motor_interface"),
                        "Could not write command: " << _command);
    return false;
  }
  if (static_cast<std::size_t>(n_written) != buffer.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_interface"),
                 "Should write %lu bytes but have written %d", buffer.size(),
                 n_written);
    return false;
  }
  return true;
}

std::optional<std::string> Motor::ReadLine(int _timeout_ms) {
  char buffer[64];
  int i = 0;
  const auto start = std::chrono::high_resolution_clock::now();
  while (i < 63) {
    int n_bytes = read(port_, buffer + i, 1);
    i += n_bytes;
    if (buffer[i - 1] == '\n') {
      buffer[n_bytes] = 0;
      return std::string(buffer);
    }
    const auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    int time_elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    if (time_elapsed_ms >= _timeout_ms) {
      return std::nullopt;
    }
    std::timespec t;
    t.tv_sec = 0;
    t.tv_nsec = 500000;
    nanosleep(&t, NULL);
  }
  return std::nullopt;
}

std::optional<std::string> Motor::ReadAnswer() {
  char buffer[64];
  struct pollfd pfd;
  pfd.fd = port_;
  pfd.events = POLLIN;
  pfd.revents = 0;
  int ready = poll(&pfd, 1, 50);
  if (!(ready > 0)) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_interface"), "Read timed out.");
    return std::nullopt;
  }
  int n_bytes = read(port_, buffer, 63);
  if (n_bytes < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("motor_interface"),
                        "Error while reading data.");
    return std::nullopt;
  }
  if (n_bytes == 0) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("motor_interface"),
                       "Received EOF character.");
    return std::nullopt;
  }
  // read does not zero terminate c string. so we have to do it manually.
  buffer[n_bytes] = 0;

  if (buffer[n_bytes - 1] != '\n') {
    RCLCPP_ERROR(rclcpp::get_logger("motor_interface"),
                 "Received data is not terminated with newline: %s", buffer);
    return std::nullopt;
  }

  std::string answer{buffer};
  while (answer.length() && (answer.back() == '\r' || answer.back() == '\n')) {
    answer.pop_back();
  }
  return answer;
}

std::optional<int> Motor::GetInt(const std::string &command) {
  if (!SendCommand(command)) {
    return std::nullopt;
  }
  auto answer = ReadAnswer();
  if (!answer) {
    return std::nullopt;
  }
  try {
    return std::stoi(*answer);
  } catch (const std::invalid_argument &e) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_interface"),
                 "Could not parse integer: '%s'", answer->c_str());
    return std::nullopt;
  }
}

bool Motor::SetHome(int position) {
  return SendCommand(cmd::kSetHome, position);
}

bool Motor::Enable() { return SendCommand(cmd::kEnable); }

bool Motor::Disable() { return SendCommand(cmd::kDisable); }

std::optional<int> Motor::GetPosition() { return GetInt(cmd::kGetPosition); }

bool Motor::SetAbsolutePositionTarget(int position) {
  return SendCommand(cmd::kSetAbsolutePositionTarget, position);
}

bool Motor::SetRelativePositionTarget(int position) {
  return SendCommand(cmd::kSetRelativePositionTarget, position);
}

bool Motor::SetVelocity(int velocity) {
  return SendCommand(cmd::kSetVelocity, velocity);
}

std::optional<int> Motor::GetVelocity() { return GetInt(cmd::kGetVelocity); }

std::optional<int> Motor::GetVelocityTarget() {
  return GetInt(cmd::kGetVelocityTarget);
}

bool Motor::MoveToCurrentTargetPosition() { return SendCommand(cmd::kMove); }

bool Motor::SetVelocityLimit(int value) {
  return SendCommand(cmd::kSetVelocityLimit, value);
}

std::optional<int> Motor::GetVelocityLimit() {
  return GetInt(cmd::kGetVelocityLimit);
}

bool Motor::SetAccelerationLimit(int value) {
  return SendCommand(cmd::kSetAccelLimit, value);
}

std::optional<int> Motor::GetAccelerationLimit() {
  return GetInt(cmd::kGetAccelLimit);
}

bool Motor::StartHoming() { return SendCommand(cmd::kStartHoming); }

std::optional<MotorStatus> Motor::PopulateMotorStatus() {
  MotorStatus motor_status;
  {
    auto value = IsPositionReached();
    if (!value) {
      return std::nullopt;
    }
    motor_status.position_reached = *value;
  }

  {
    auto value = IsHoming();
    if (!value) {
      return std::nullopt;
    }
    motor_status.homing = *value;
  }
  {
    auto value = IsEnabled();
    if (!value) {
      return std::nullopt;
    }
    motor_status.enabled = *value;
  }
  {
    auto value = GetLowerLimitSwitch();
    if (!value) {
      return std::nullopt;
    }
    motor_status.limit_switches.lower_pressed = *value;
  }
  {
    auto value = GetUpperLimitSwitch();
    if (!value) {
      return std::nullopt;
    }
    motor_status.limit_switches.upper_pressed = *value;
  }
  return motor_status;
}

}  // namespace gantry
