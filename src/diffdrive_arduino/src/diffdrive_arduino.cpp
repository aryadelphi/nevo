#include "diffdrive_arduino/diffdrive_arduino.h"


#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <cmath>



DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino"))
{}


hardware_interface::CallbackReturn DiffDriveArduino::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");


  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  if (info_.hardware_parameters.count("min_counts_per_frame"))
    cfg_.min_counts_per_frame = std::stof(info_.hardware_parameters["min_counts_per_frame"]);
  if (info_.hardware_parameters.count("zero_counts_per_frame"))
    cfg_.zero_counts_per_frame = std::stof(info_.hardware_parameters["zero_counts_per_frame"]);
  RCLCPP_INFO(logger_, "Deadband compensation: min %.2f counts/frame (0 = off), zero below %.2f",
    cfg_.min_counts_per_frame, cfg_.zero_counts_per_frame);

  // Set up the wheels
  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  // Set up the Arduino
  arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);  

  RCLCPP_INFO(logger_, "Finished Configuration");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr> DiffDriveArduino::on_export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> DiffDriveArduino::on_export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

  command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  return command_interfaces;
}


hardware_interface::CallbackReturn DiffDriveArduino::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Starting Controller...");

  arduino_.sendEmptyMsg();
  // arduino.setPidValues(9,7,0,100);
  // arduino.setPidValues(14,7,0,100);
  arduino_.setPidValues(30, 20, 0, 100);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduino::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Stopping Controller...");

  // Stop the motors now instead of waiting for the firmware auto-stop.
  if (arduino_.connected())
  {
    arduino_.setMotorValues(0, 0);
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduino::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Use the controller_manager period (monotonic) rather than wall-clock time,
  // which jumps when the system clock is set or synced.
  const double deltaSeconds = period.seconds();

  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  int l_enc = l_wheel_.enc, r_enc = r_wheel_.enc;
  if (!arduino_.readEncoderValues(l_enc, r_enc))
  {
    // Keep the last position and velocity for this cycle instead of
    // publishing a bogus jump; report persistent failures.
    if (++enc_read_failures_ % 20 == 0)
    {
      RCLCPP_WARN(logger_, "Encoder read failed %d times in a row", enc_read_failures_);
    }
    return return_type::OK;
  }
  enc_read_failures_ = 0;
  l_wheel_.enc = l_enc;
  r_wheel_.enc = r_enc;

  if (deltaSeconds <= 0.0)
  {
    return return_type::OK;
  }

  double pos_prev = l_wheel_.pos;
  l_wheel_.pos = l_wheel_.calcEncAngle();
  l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = r_wheel_.pos;
  r_wheel_.pos = r_wheel_.calcEncAngle();
  r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;

  return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  // Counts per firmware PID frame (cfg_.loop_rate must match the firmware PID_RATE).
  // Round instead of truncating toward zero. Deadband compensation (2026-10-27, deadband_test.py):
  // the firmware PID does not turn a wheel below ~3.5-4 counts/frame (0.02 m/s / 0.10 rad/s
  // robot commands did nothing, 0.03 / 0.12 moved), so small non-zero targets are raised to
  // the minimum that moves. Odometry still comes from the encoders, so the loop stays closed.
  const auto counts = [this](double cmd_rad_s, double rads_per_count) {
    double c = cmd_rad_s / rads_per_count / cfg_.loop_rate;
    if (cfg_.min_counts_per_frame > 0.0f) {
      if (std::abs(c) < cfg_.zero_counts_per_frame) {
        c = 0.0;
      } else if (std::abs(c) < cfg_.min_counts_per_frame) {
        c = std::copysign(static_cast<double>(cfg_.min_counts_per_frame), c);
      }
    }
    return static_cast<int>(std::lround(c));
  };
  arduino_.setMotorValues(counts(l_wheel_.cmd, l_wheel_.rads_per_count), counts(r_wheel_.cmd, r_wheel_.rads_per_count));




  return return_type::OK;


  
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveArduino,
  hardware_interface::SystemInterface
)