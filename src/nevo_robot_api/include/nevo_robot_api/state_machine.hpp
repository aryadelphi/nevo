#pragma once

#include <string>

namespace nevo_robot_api
{

enum class RobotMode {
  IDLE,
  MOVING,
  MAPPING,
  CHARGING,
  PAUSED,
  ERROR,
  INTERRUPTED
};

class StateMachine
{
public:
  StateMachine();

  /// Set mode dari string (IDLE, MOVING, dst).
  /// Return false kalau mode gak valid atau transisi ilegal.
  bool setMode(const std::string & mode_str);

  RobotMode getMode() const;
  std::string modeToString(RobotMode m) const;

private:
  RobotMode current_mode_;

  bool isValidTransition(RobotMode from, RobotMode to);
};

}  // namespace nevo_robot_api
