#pragma once

namespace gantry {
struct LimitSwitchesStatus {
  bool lower_pressed;
  bool upper_pressed;
};
struct MotorStatus {
  bool homing;
  LimitSwitchesStatus limit_switches;
  bool enabled;
  bool position_reached;
};
}  // namespace gantry
