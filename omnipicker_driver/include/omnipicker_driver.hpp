#ifndef OMNIPICKER_HPP_
#define OMNIPICKER_HPP_
#include <rclcpp/rclcpp.hpp>
#include "omnipicker_interfaces/srv/omni_picker_control.hpp"
#include "omnipicker_interfaces/msg/omni_picker_state.hpp"

#include <boost/asio.hpp>

using namespace std;

const unsigned char can_param[22] = {
    0x49, 0x3B, 0x42, 0x57, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x45, 0x2E}; // set can0 bitrate 1000kbps 5000kbps

const unsigned char open_can[22] = {
    0x49, 0x3B, 0x44, 0x57, 0x01, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x45, 0x2E}; // open can0

const unsigned char default_open[16] = {
    0x5A, 0x08, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xA5}; // default open

const unsigned char default_clamp[16] = {
    0x5A, 0x08, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xA5}; // default clamp

class SerialPort
{
public:
  using DataCallback = std::function<void(const unsigned char *, size_t)>;
  SerialPort(rclcpp::Logger logger);
  ~SerialPort();

  bool connect(const string &port);
  bool sendHexData(const unsigned char *data, size_t size);
  void setDataCallback(DataCallback callback);
private:
  void asyncRead();

private:
  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_port_{io_service_};
  std::thread io_thread_;
  std::vector<unsigned char> read_buffer_;
  std::mutex serial_mutex_;
  DataCallback data_callback_;
  rclcpp::Logger logger_;
};

class OmniPickerRos2 : public rclcpp::Node
{
public:
  OmniPickerRos2();
  bool OmniPickerConnect();

private:
  void timerCallback();
  void controlServiceCallback(const std::shared_ptr<omnipicker_interfaces::srv::OmniPickerControl::Request> req, std::shared_ptr<omnipicker_interfaces::srv::OmniPickerControl::Response> res);

private:
  void parseSerialData(const unsigned char* data, size_t size);
  void parseHeartbeatData(const unsigned char* data, size_t size);
  void parseOmniPickerStateData(const unsigned char* data, size_t size);

private:
  bool OmniPickerDefaultOpenAndClamp(const string &cmd);
  bool OmniPickerCustomOpenAndClamp(const float &pos, const float &force, const float &vel, const float &acc, const float &dec);

private:
  string charToHexStr(const unsigned char data);

private:
  std::unique_ptr<SerialPort> serial_port_;
  std::mutex state_mutex_;
  omnipicker_interfaces::msg::OmniPickerState state_;

  rclcpp::Service<omnipicker_interfaces::srv::OmniPickerControl>::SharedPtr control_service_;
  rclcpp::Publisher<omnipicker_interfaces::msg::OmniPickerState>::SharedPtr state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // OMNIPICKER_HPP_
