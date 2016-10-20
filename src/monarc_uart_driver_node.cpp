#include <sstream>
#include <stdexcept>
#include <string>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"

#include "uart_handler.h"
#include "api.pb.h"

void uart_reader(UartHandler* uart) {
  ros::NodeHandle nh("~");

  /*
   * Advertise on a group of topics.
   */
  ros::Publisher atmospheric_pressure_pub = nh.advertise<std_msgs::Int32>("atmospheric_pressure", 1000);

  while (ros::ok()) {
    /*
     * Read from UART.
     */
    std::string data = uart->read();
    if (data.length() == 0) {
      ROS_INFO("I heard nothing");
      continue;
    }

    /*
     * Parse UART message to protobuf.
     */
    monarcpb::SysCtrlToNavCPU message;
    if (!message.ParseFromString(data)) {
      ROS_INFO("I heard something I could not parse [%s]", data.c_str());
      continue;
    }

    /*
     * Distribute information from protobuf to ROS topics.
     */
    if (message.has_telemetry()) {
      const monarcpb::SysCtrlToNavCPU_Telemetry telemetry = message.telemetry();

      std_msgs::Int32 atmospheric_pressure;
      atmospheric_pressure.data = telemetry.atmospheric_pressure();
      atmospheric_pressure_pub.publish(atmospheric_pressure);
    }
  }
}

/*
 * nav_cpu_state is populated partially by each of the following callbacks.
 */
monarcpb::NavCPUToSysCtrl nav_cpu_state;

void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& navSatFix) {
  monarcpb::NavCPUToSysCtrl_Telemetry* telemetry = nav_cpu_state.mutable_telemetry();
  monarcpb::GPSFix* gps = telemetry->mutable_gps();
  switch (navSatFix->status.status) {
    case sensor_msgs::NavSatStatus::STATUS_NO_FIX:
      gps->set_status(monarcpb::GPSFix_Status_NO_FIX);
      break;
    case sensor_msgs::NavSatStatus::STATUS_FIX:
      gps->set_status(monarcpb::GPSFix_Status_FIX);
      break;
    case sensor_msgs::NavSatStatus::STATUS_SBAS_FIX:
      gps->set_status(monarcpb::GPSFix_Status_SBAS_FIX);
      break;
    case sensor_msgs::NavSatStatus::STATUS_GBAS_FIX:
      gps->set_status(monarcpb::GPSFix_Status_GBAS_FIX);
      break;
    default:
      throw std::invalid_argument("unexpected NavSatStatus value");
  }
  gps->set_latitude(navSatFix->latitude);
  gps->set_longitude(navSatFix->longitude);
  gps->set_altitude(navSatFix->altitude);
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
  ros::Subscriber sub = nh.subscribe("fix", 1, gpsFixCallback);

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
