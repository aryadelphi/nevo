#include "nevo_robot_api/state_machine.hpp"

namespace nevo_robot_api
{

StateMachine::StateMachine()
: current_mode_(RobotMode::IDLE)
{
}

RobotMode StateMachine::getMode() const
{
  return current_mode_;
}

std::string StateMachine::modeToString(RobotMode m) const
{
  switch (m) {
    case RobotMode::IDLE:     return "IDLE";
    case RobotMode::MOVING:   return "MOVING";
    case RobotMode::MAPPING:  return "MAPPING";
    case RobotMode::CHARGING: return "CHARGING";
    case RobotMode::PAUSED:   return "PAUSED";
    case RobotMode::INTERRUPTED:   return "INTERRUPTED";
    case RobotMode::ERROR:
    default:                  return "ERROR";
  }
}

bool StateMachine::setMode(const std::string & mode_str)
{
  RobotMode target;

  if (mode_str == "IDLE")          target = RobotMode::IDLE;
  else if (mode_str == "MOVING")   target = RobotMode::MOVING;
  else if (mode_str == "MAPPING")  target = RobotMode::MAPPING;
  else if (mode_str == "CHARGING") target = RobotMode::CHARGING;
  else if (mode_str == "PAUSED")   target = RobotMode::PAUSED;
  else if (mode_str == "INTERRUPTED")   target = RobotMode::INTERRUPTED;
  else                             return false;  // unknown mode

  if (!isValidTransition(current_mode_, target)) {
    return false;
  }

  current_mode_ = target;
  return true;
}

bool StateMachine::isValidTransition(RobotMode from, RobotMode to)
{
  // Aturan sementara, bisa lu perketat nanti.

  // Dari ERROR: tidak boleh kemana-mana
  if (from == RobotMode::ERROR) {
    return false;
  }

  // Dari CHARGING: tidak langsung MOVING
  if (from == RobotMode::CHARGING && to == RobotMode::MOVING) {
    return false;
  }

  if (from == RobotMode::PAUSED &&
      to != RobotMode::IDLE &&
      to != RobotMode::MOVING) {
    return false;
  }


  if(from == RobotMode::IDLE && to == RobotMode::INTERRUPTED) {
    return false;
  }

  if(from == RobotMode::INTERRUPTED && to == RobotMode::INTERRUPTED) {
    return false;
  }

  if(from == RobotMode::MOVING && to == RobotMode::INTERRUPTED) {
    return true;
  }

  if(from == RobotMode::INTERRUPTED && to == RobotMode::MOVING) {
    return true;
  }

  if(from == RobotMode::INTERRUPTED && to == RobotMode::IDLE) {
    return true;
  }

  // Selain aturan di atas, anggap valid
  return true;
}

}  // namespace nevo_robot_api
