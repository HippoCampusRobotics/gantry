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

#include <hippo_common/param_utils.hpp>

#include "gantry/motor_component/motor_component.hpp"

namespace gantry {

void MotorNode::InitParams() {
  HIPPO_COMMON_DECLARE_PARAM_READONLY(type);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(device);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(timeout);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(update_period_ms);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(increments_per_length_unit);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(increments_per_rev);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(max_rpm);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(defaults.max_rpm);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(defaults.max_accel);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(defaults.max_decel);
}

}  // namespace gantry
