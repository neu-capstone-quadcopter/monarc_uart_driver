#include <sstream>
#include <string>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "uart_handler.h"
#include "api.pb.h"

void uart_reader(UartHandler* uart) {
  ros::NodeHandle nh("~");

  /*
   * Advertise on a group of topics.
   */
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("testing", 1000);

  while (ros::ok()) {
    /*
     * Read from UART.
     */
    std::string data = uart->read();
    if (data.length() == 0) {
      continue;
    }

    /*
     * Parse UART message to protobuf.
     */
    monarcpb::SysCtrlToNavCPU message;
    if (!message.ParseFromString(data)) {
      continue;
    }
    ROS_INFO("Received from UART: %s", message.DebugString().c_str());

    /*
     * Distribute information from protobuf to ROS topics.
     */
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world! " << " " << uart->isOpen() << "...";
    message.SerializeToOstream(&ss);
    msg.data = ss.str();

    chatter_pub.publish(msg);
  }
}

/*
 * nav_cpu_state is populated partially by each of the following callbacks.
 */
monarcpb::NavCPUToSysCtrl nav_cpu_state;

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

std::string get_uart_port() {
  ros::NodeHandle param_handle("~");

  std::string port;
  param_handle.getParam("uart_port", port);
  return port;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "monarc_uart_driver");
  ros::NodeHandle nh;

  /**
   * Initialize the uart handler with the provided port.
   */
  std::string port = get_uart_port();
  UartHandler uart(port);
  if (!uart.isOpen()) {
    throw serial::PortNotOpenedException(port.c_str());
  }
  ROS_INFO("monarc_uart_driver using UART port: %s", port.c_str());

  /*
   * Launch UART reader thread.
   */
  boost::thread reader_thread(uart_reader, &uart);

  /*
   * Subscribe to all necessary topics, each with a queue_size of 1
   */
  ros::Subscriber sub = nh.subscribe("chatter", 1, chatterCallback);

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    /*
     * Process all callbacks, which will populate nav_cpu_state.
     */
    ros::spinOnce();

    /*
     * Send nav_cpu_state over UART.
     */
    std::string state_data;
    nav_cpu_state.SerializeToString(&state_data);
    uart.write(state_data);

    /*
     * Sleep for the remainder of the interval.
     */
    loop_rate.sleep();
  }

  reader_thread.join();
  return 0;
}
