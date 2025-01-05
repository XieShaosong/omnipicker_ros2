#include "omnipicker_driver.hpp"

OmniPickerRos2::OmniPickerRos2() : Node("OmniPicker_Ros2")
{
  this->declare_parameter("omnipicker_port", "/dev/ttyACM0");
  serial_port_ = std::make_unique<SerialPort>(this->get_logger());
  serial_port_->setDataCallback(std::bind(&OmniPickerRos2::parseSerialData, this, std::placeholders::_1, std::placeholders::_2));

  control_service_ = this->create_service<omnipicker_interfaces::srv::OmniPickerControl>("/omnipicker_control",
    std::bind(&OmniPickerRos2::controlServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  state_publisher_ = this->create_publisher<omnipicker_interfaces::msg::OmniPickerState>("/omnipicker_state", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&OmniPickerRos2::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "OmniPickerRos2 node started");
}

void OmniPickerRos2::timerCallback()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_publisher_->publish(state_);
}

void OmniPickerRos2::controlServiceCallback(const std::shared_ptr<omnipicker_interfaces::srv::OmniPickerControl::Request> req,
                                            std::shared_ptr<omnipicker_interfaces::srv::OmniPickerControl::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "Received mode: %s", req->mode.c_str());
  if (req->mode == "auto")
    res->result = OmniPickerDefaultOpenAndClamp(req->cmd);
  else if (req->mode == "manual")
     res->result = OmniPickerCustomOpenAndClamp(req->pos, req->force, req->vel, req->acc, req->dec);
  else
    res->result = false;

  res->result_code = res->result ? "success" : "failed";
}

bool OmniPickerRos2::OmniPickerConnect()
{
  string port;
  if (this->get_parameter("omnipicker_port", port))
  {
    RCLCPP_INFO(this->get_logger(), "OmniPicker port: %s", port.c_str());
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get OmniPicker port");
    return false;
  }

  if (!serial_port_->connect(port))
    return false;

  //set can0 bitrate 1000kbps 5000kbps
  bool ret;
  ret = serial_port_->sendHexData(can_param, sizeof(can_param));

  // open can0
  ret = serial_port_->sendHexData(open_can, sizeof(open_can));

  if (ret)
    RCLCPP_INFO(this->get_logger(), "OmniPicker connected");
  else
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to OmniPicker");

  return ret;
}

bool OmniPickerRos2::OmniPickerDefaultOpenAndClamp(const string &cmd)
{
  if (cmd == "open")
    return serial_port_->sendHexData(default_open, sizeof(default_open));
  else if (cmd == "clamp")
    return serial_port_->sendHexData(default_clamp, sizeof(default_clamp));
  else
    return false;
}

bool OmniPickerRos2::OmniPickerCustomOpenAndClamp(const float &pos, const float &force, const float &vel, const float &acc, const float &dec)
{
  unsigned char custom_data[16] = {
    0x5A, 0x08, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xA5};
  
  custom_data[8] = static_cast<unsigned char>(pos * 255);
  custom_data[9] = static_cast<unsigned char>(force * 255);
  custom_data[10] = static_cast<unsigned char>(vel * 255);
  custom_data[11] = static_cast<unsigned char>(acc * 255);
  custom_data[12] = static_cast<unsigned char>(dec * 255);

  return serial_port_->sendHexData(custom_data, sizeof(custom_data));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto omnipicker = std::make_shared<OmniPickerRos2>();

  if (!omnipicker->OmniPickerConnect())
  {
    return -1;  
  }
  
  rclcpp::spin(omnipicker);
  rclcpp::shutdown();
  return 0;
}
