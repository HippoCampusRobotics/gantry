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

#include "gantry/motor/mcbl3006.hpp"

namespace gantry {

namespace cmd {
static constexpr char kGetOperatingStatus[] = "OST";
static constexpr char kGetConfigurationStatus[] = "CST";
static constexpr char kGetType[] = "GTYP";
static constexpr char kGetSerial[] = "GSER";
static constexpr char kGetIOConfig[] = "IOC";
}  // namespace cmd

bool MCBL3006::SetDecelerationLimit(int value) {
  return SendCommand(cmd::kSetDecelLimit, value);
}

std::optional<int> MCBL3006::GetDecelerationLimit() {
  return GetInt(cmd::kGetDecelLimit);
}

std::optional<bool> MCBL3006::IsHoming() {
  return static_cast<bool>(operating_status_ & (1 << 0));
}

std::optional<bool> MCBL3006::GetLowerLimitSwitch() {
  return static_cast<bool>(operating_status_ & (1 << 8));
}

std::optional<bool> MCBL3006::GetUpperLimitSwitch() {
  return static_cast<bool>(operating_status_ & (1 << 9));
}

std::optional<bool> MCBL3006::IsEnabled() {
  return static_cast<bool>(configuration_status_ & (1 << 10));
}

std::optional<bool> MCBL3006::IsPositionReached() {
  return static_cast<bool>(operating_status_ & (1 << 16));
}

std::optional<int> MCBL3006::ReadConfigurationStatus() {
  return GetInt(cmd::kGetConfigurationStatus);
}

std::optional<int> MCBL3006::ReadOperatingStatus() {
  return GetInt(cmd::kGetOperatingStatus);
}

std::optional<MotorStatus> MCBL3006::UpdateStatus() {
  auto operating_status = ReadOperatingStatus();
  if (!operating_status) {
    return std::nullopt;
  }
  auto configuration_status = ReadConfigurationStatus();
  if (!configuration_status) {
    return std::nullopt;
  }
  operating_status_ = *operating_status;
  configuration_status_ = *configuration_status;
  return PopulateMotorStatus();
}

}  // namespace gantry
