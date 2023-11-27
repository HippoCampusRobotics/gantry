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
}  // namespace cmd

std::optional<bool> MCBL3006::IsHoming() {
  auto answer = GetInt(cmd::kGetOperatingStatus);
  if (!answer) {
    return std::nullopt;
  }
  return static_cast<bool>(*answer & (1 << 0));
}

std::optional<bool> MCBL3006::GetLowerLimitSwitch() {
  auto answer = GetInt(cmd::kGetOperatingStatus);
  if (!answer) {
    return std::nullopt;
  }
  return static_cast<bool>(*answer & (1 << 8));
}

std::optional<bool> MCBL3006::GetUpperLimitSwitch() {
  auto answer = GetInt(cmd::kGetOperatingStatus);
  if (!answer) {
    return std::nullopt;
  }
  return static_cast<bool>(*answer & (1 << 9));
}

std::optional<bool> MCBL3006::IsEnabled() {
  auto answer = GetInt(cmd::kGetConfigurationStatus);
  if (!answer) {
    return std::nullopt;
  }
  return static_cast<bool>(*answer & (1 << 10));
}

std::optional<bool> MCBL3006::IsPositionReached() {
  auto answer = GetInt(cmd::kGetOperatingStatus);
  if (!answer) {
    return std::nullopt;
  }
  return static_cast<bool>(*answer & (1 << 16));
}

}  // namespace gantry
