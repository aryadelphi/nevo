#include "diffdrive_arduino/arduino_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>
#include <cerrno>
#include <chrono>
#include <thread>


void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();
    // Opening the port resets a Nano/Uno (DTR); give the bootloader time
    // before the first command, now that the read timeout is short.
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));

}


void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}

bool ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
    std::string response = sendMsg("e\r");

    // Expect "<int> <int>\r\n". A timeout returns an empty or partial line;
    // parsing that with atoi would yield 0 and make the odometry jump.
    const char *p = response.c_str();
    char *end = nullptr;
    errno = 0;
    long v1 = std::strtol(p, &end, 10);
    if (end == p || errno != 0) return false;
    p = end;
    long v2 = std::strtol(p, &end, 10);
    if (end == p || errno != 0) return false;
    while (*end == ' ' || *end == '\r' || *end == '\n') ++end;
    if (*end != '\0' || response.find('\n') == std::string::npos) return false;

    val_1 = static_cast<int>(v1);
    val_2 = static_cast<int>(v2);
    return true;
}

void ArduinoComms::setMotorValues(int val_1, int val_2)
{
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    sendMsg(ss.str(), false);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    sendMsg(ss.str());
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    serial_conn_.flushInput();
    serial_conn_.write(msg_to_send);
    std::string response = serial_conn_.readline();

    if (print_output)
    {
        // RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
        // RCLCPP_INFO_STREAM(logger_,"Received: " << response);
    }

    return response;
}