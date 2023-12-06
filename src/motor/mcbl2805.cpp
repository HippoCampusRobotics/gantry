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

#include "gantry/motor/mcbl2805.hpp"

#include <rclcpp/rclcpp.hpp>

namespace gantry {

namespace cmd {
static constexpr char kGetStatus[] = "GST";
static constexpr char kGetActualStatus[] = "GAST";
}  // namespace cmd

static constexpr std::size_t kStatusSize = 7;
static constexpr std::size_t kActualStatusSize = 4;

std::optional<MotorStatus> MCBL2805::UpdateStatus() {
  auto actual_status = ReadActualStatus();
  auto status = ReadStatus();
  if (!actual_status) {
    return std::nullopt;
  }
  if (!status) {
    return std::nullopt;
  }
  actual_status_ = *actual_status;
  status_ = *status;
  return PopulateMotorStatus();
}

std::optional<bool> MCBL2805::IsHoming() { return IsBitSet(actual_status_, 3); }

std::optional<bool> MCBL2805::GetLowerLimitSwitch() {
  return IsBitSet(status_, 6);
}

std::optional<bool> MCBL2805::GetUpperLimitSwitch() {
  return IsBitSet(actual_status_, 0);
}

std::optional<bool> MCBL2805::IsEnabled() { return IsBitSet(status_, 3); }

std::optional<bool> MCBL2805::IsPositionReached() {
  return IsBitSet(status_, 4);
}

std::optional<bool> MCBL2805::IsBitSet(const std::string &data, std::size_t i) {
  if (i >= data.size()) {
    return std::nullopt;
  }
  if (data[i] == '0') {
    return false;
  } else if (data[i] == '1') {
    return true;
  }
  return std::nullopt;
}

std::optional<std::string> MCBL2805::ReadStatus() {
  if (!SendCommand(cmd::kGetStatus)) {
    return std::nullopt;
  }
  auto answer = ReadAnswer();
  if (!answer) {
    return std::nullopt;
  }
  if (answer->size() != kStatusSize) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("mcbl2805"),
                        "GetStatus returned wrong data length: "
                            << std::to_string(answer->size()) << " (expected "
                            << std::to_string(kStatusSize) << ")");
    return std::nullopt;
  }
  return answer;
}

std::optional<std::string> MCBL2805::ReadActualStatus() {
  if (!SendCommand(cmd::kGetActualStatus)) {
    return std::nullopt;
  }
  auto answer = ReadAnswer();
  if (!answer) {
    return std::nullopt;
  }
  if (answer->size() != kActualStatusSize) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("mcbl2805"),
                        "GetActualStatus returned wrong data length: "
                            << std::to_string(answer->size()) << " (expected "
                            << std::to_string(kStatusSize) << ")");
    return std::nullopt;
  }
  return answer;
}

}  // namespace gantry
