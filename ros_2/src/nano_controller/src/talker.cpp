#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <boost/asio.hpp>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;
using namespace boost::asio;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mec_cont/reference", 10);
      //geometry_msgs::msg::TwistStamped
      // timer_ = this->create_wall_timer(
      // 500ms, std::bind(&MinimalPublisher::timer_callback, this));

      timer_callback();
    }

  private:
    void timer_callback()
    {
      // Initialize IO context
      io_service io;

      // Open the serial port (adjust for your port, e.g., COM3 or /dev/ttyUSB0)
      serial_port serial(io, "/dev/ttyUSB0");  // Adjust the port to your system
      serial.set_option(serial_port::baud_rate(9600));
      serial.set_option(serial_port::character_size(8));
      serial.set_option(serial_port::parity(serial_port::parity::none));
      serial.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
      serial.set_option(serial_port::flow_control(serial_port::flow_control::none));

      // Data Pre-Processing
      char delimiter = '@';
      std::string temp_str;  // Temporary string to hold each substring


      // Read and print incoming data from Arduino
      while (true) {
          boost::asio::streambuf buf;  // Buffer to store incoming data
          boost::asio::read_until(serial, buf, '\n');  // Read until newline character
          
          std::istream is(&buf);
          std::string line;
          std::getline(is, line);  // Read the data into a string
          
          std::cout << "Received: " << line << std::endl; 

          // Publissh to Contnroller
          std::vector<float> values;  // Vector to store the split float values      
          std::stringstream ss(line);  // Create a stringstream from the input string
          std::string temp;  // Temporary string to hold each substring

          while (std::getline(ss, temp, delimiter)) {
            values.push_back(std::stof(temp));  // Convert substring to float and store in vector
          }

          // Fill vvvavlues for publishing
          //geometry_msgs::msg::TwistStamped msg;
          auto msg = geometry_msgs::msg::TwistStamped();

          msg.header.stamp = this->get_clock()->now();
          msg.header.frame_id = "base_link"; 

          // Set linear velocity (x, y, z) and angular velocity (x, y, z)
          msg.twist.linear.x = ((515 - values[0] )/515) * 6.0  ;  
          msg.twist.linear.y = (((506 - values[1] )/506) * 6.0) * -1;   // Here multiplied with -1 becaue the controller respont to y commands inversly.
          msg.twist.linear.z = 0.0;

          msg.twist.angular.x = 0.0;
          msg.twist.angular.y = 0.0;
          msg.twist.angular.z = (((510 - values[4])/510) * 6.0) * -1;   // Here multiplied with -1 becaue the controller respont to y commands inversly.  

          RCLCPP_INFO(this->get_logger(), "Publishing TwistStamped message with linear: (%.2f, %.2f, %.2f) and angular: (%.2f, %.2f, %.2f)",
                    msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                    msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z);

          publisher_->publish(msg);

        // std::cout << "Received: " << line << std::endl;  // Print the received data
      }
    }
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
