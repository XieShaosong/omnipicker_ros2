#include "omnipicker_driver.hpp"

SerialPort::SerialPort(rclcpp::Logger logger) : logger_(logger)
{
  read_buffer_.resize(1024);
}

SerialPort::~SerialPort()
{
  io_service_.stop();
  if (io_thread_.joinable())
    io_thread_.join();
}

bool SerialPort::connect(const string &port)
{
  try
  {
    serial_port_.open(port);
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(115200));
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  }
  catch (boost::system::system_error &e)
  {
    RCLCPP_ERROR(logger_, "Failed to open serial port %s: %s", port.c_str(), e.what());
    return false;
  }

  asyncRead();
  io_thread_ = std::thread([this]() { while (true) { io_service_.run(); }});

  RCLCPP_INFO(logger_, "Connected to serial port %s", port.c_str());
  return true;
}

void SerialPort::setDataCallback(DataCallback callback)
{
  data_callback_ = callback;
}

bool SerialPort::sendHexData(const unsigned char *data, size_t size)
{
  std::stringstream ss;
  for (size_t i = 0; i < size; i++)
  {
    ss << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<int>(data[i]) << " ";
  }
  RCLCPP_INFO(logger_, "send %zu bytes via serial: %s", size, ss.str().c_str());

  try
  {
    boost::asio::write(serial_port_, boost::asio::buffer(data, size));
  }
  catch (boost::system::system_error &e)
  {
    RCLCPP_ERROR(logger_, "Failed to write to serial port: %s", e.what());
    return false;
  }
  return true;
}

void SerialPort::asyncRead()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  serial_port_.async_read_some(boost::asio::buffer(read_buffer_), 
    [this](const boost::system::error_code &error, std::size_t n)
    {
      if (error)
      {
        RCLCPP_ERROR(logger_, "Failed to read from serial port: %s", error.message().c_str());
      }
      else
      {
        if (data_callback_)
          data_callback_(read_buffer_.data(), n);

        asyncRead();
      }
    });
}
