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

#include "gantry/motor//motor_interface.hpp"

namespace gantry {
class MCBL3006 : public Motor {
 public:
  std::optional<MotorStatus> UpdateStatus() override;
  bool SetDecelerationLimit(int value) override;
  std::optional<int> GetDecelerationLimit() override;

 protected:
  std::optional<bool> IsHoming() override;
  std::optional<bool> GetLowerLimitSwitch() override;
  std::optional<bool> GetUpperLimitSwitch() override;
  std::optional<bool> IsEnabled() override;
  std::optional<bool> IsPositionReached() override;

 private:
  std::optional<int> ReadOperatingStatus();
  std::optional<int> ReadConfigurationStatus();

  int operating_status_;
  int configuration_status_;
};
}  // namespace gantry
