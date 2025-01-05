#include "omnipicker_driver.hpp"

string OmniPickerRos2::charToHexStr(const unsigned char data)
{
  std::stringstream ss;
  ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(data);
  return ss.str();
}

void OmniPickerRos2::parseSerialData(const unsigned char* data, size_t size)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (data[0] == 0x5A && data[1] == 0xFF)
    parseHeartbeatData(data, size);
  else if (data[0] == 0x5A && data[1] == 0x08)
    parseOmniPickerStateData(data, size);
}

void OmniPickerRos2::parseHeartbeatData(const unsigned char* data, size_t size)
{
  if (size != 17 && data[0] != 0x5A && data[1] != 0xFF)
    return;

  unsigned char status = data[10];
  unsigned char can0 = (status & 0x20) >> 5;
  // unsigned char can1 = (status & 0x10) >> 4;
  unsigned char can0_state = (status & 0x0C) >> 2;
  // unsigned char can1_state = status & 0x03;

  state_.can0_flag = static_cast<int>(can0);
  state_.can0_state = static_cast<int>(can0_state);
}

void OmniPickerRos2::parseOmniPickerStateData(const unsigned char* data, size_t size)
{
  if (data[0] != 0x5A && data[1] != 0x08)
    return;

  state_.raw_data.clear();
  std::stringstream ss;
  for (size_t i = 0; i < size; i++)
  {
    ss << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<int>(data[i]) << " ";

    state_.raw_data.push_back(data[i]);
  }
  RCLCPP_INFO(this->get_logger(), "Read %zu bytes via serial: %s", size, ss.str().c_str());
  
  unsigned char error = data[7];
  unsigned char state = data[8];
  unsigned char pos = data[9];
  unsigned char vel = data[10];
  unsigned char force = data[11];

  state_.picker_fault_code = charToHexStr(error);
  state_.picker_state = charToHexStr(state);

  if (state_.picker_fault_code != "00")
    RCLCPP_ERROR(this->get_logger(), "OmniPicker fault code: %s", state_.picker_fault_code.c_str());

  unsigned char max = 0xFF;
  state_.rt_pos = static_cast<float>(pos) / static_cast<float>(max);
  state_.rt_vel = static_cast<float>(vel) / static_cast<float>(max);
  state_.rt_force = static_cast<float>(force) / static_cast<float>(max);
}
