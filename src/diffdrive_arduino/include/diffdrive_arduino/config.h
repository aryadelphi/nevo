#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>


struct Config
{
  std::string left_wheel_name = "left_wheel";
  std::string right_wheel_name = "right_wheel";
  float loop_rate = 30;
  std::string device = "/dev/ttyUSB0";
  int baud_rate = 57600;
  int timeout = 1000;
  int enc_counts_per_rev = 2000;
  // Motor deadband compensation per wheel, in encoder counts per firmware PID frame.
  // Targets below zero_counts_per_frame are sent as 0; targets between that and
  // min_counts_per_frame are raised to min_counts_per_frame (the smallest target the
  // firmware PID actually turns the wheel at). min_counts_per_frame <= 0 disables it.
  float min_counts_per_frame = 0.0f;
  float zero_counts_per_frame = 0.5f;
};


#endif // DIFFDRIVE_ARDUINO_CONFIG_H