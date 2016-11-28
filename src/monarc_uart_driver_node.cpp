#include <sstream>
#include <stdexcept>
#include <string>
#include <boost/thread.hpp>

#include "tf/transform_datatypes.h"

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"
#include "monarc_uart_driver/FlightControl.h"
#include "monarc_uart_driver/NavCommand.h"

#include "uart_handler.h"
#include "api.pb.h"

void uart_reader(UartHandler* uart) {
  ros::NodeHandle nh;

  /*
   * Advertise on a group of topics.
   */
  ros::Publisher atmospheric_pressure_pub = nh.advertise<std_msgs::Int32>("atmospheric_pressure", 100);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 100);
  ros::Publisher ultrasonic_altitude_pub = nh.advertise<std_msgs::Int32>("ultrasonic_altitude", 100);
  ros::Publisher nav_command_pub = nh.advertise<monarc_uart_driver::NavCommand>("nav_command", 100);

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
     * Distribute information from protobuf to ROS topics. Also grab the time for the Headers.
     */
    ros::Time time_stamp = ros::Time::now();

    if (message.has_telemetry()) {
      const monarcpb::SysCtrlToNavCPU_Telemetry telemetry = message.telemetry();

      /*
       * Read and publish atmospheric pressure
       */
      if (telemetry.atmospheric_pressure() != 0) {
          std_msgs::Int32 atmospheric_pressure;
          atmospheric_pressure.data = telemetry.atmospheric_pressure();
          atmospheric_pressure_pub.publish(atmospheric_pressure);
      }

      /*
       * Read and publish all IMU data
       */
      if (telemetry.has_accelerometer() && telemetry.has_gyroscope() && telemetry.has_magnetometer()) {
        sensor_msgs::Imu imu_data;

        imu_data.header.stamp = time_stamp;

        imu_data.orientation = tf::createQuaternionMsgFromRollPitchYaw((double) telemetry.magnetometer().x(),
                                                                     (double) telemetry.magnetometer().y(),
                                                                     (double) telemetry.magnetometer().z());
        imu_data.orientation_covariance[0] = -1;

        imu_data.angular_velocity.x = (double) telemetry.gyroscope().x();
        imu_data.angular_velocity.y = (double) telemetry.gyroscope().y();
        imu_data.angular_velocity.z = (double) telemetry.gyroscope().z();
        imu_data.angular_velocity_covariance[0] = -1;

        imu_data.linear_acceleration.x = (double) telemetry.accelerometer().x();
        imu_data.linear_acceleration.y = (double) telemetry.accelerometer().y();
        imu_data.linear_acceleration.z = (double) telemetry.accelerometer().z();
        imu_data.linear_acceleration_covariance[0] = -1;

        imu_pub.publish(imu_data);
      }

      /*
       * Read and publish ultrasonic altitude data
       */
      if (telemetry.altitude() != 0) {
        std_msgs::Int32 ultrasonic_altitude;
        ultrasonic_altitude.data = telemetry.altitude();
        ultrasonic_altitude_pub.publish(ultrasonic_altitude);
      }
    }

    if (message.has_command()) {
        const monarcpb::SysCtrlToNavCPU_NavigationCommand command = message.command();
        monarc_uart_driver::NavCommand nav_command;

        nav_command.command_location.header.stamp = time_stamp;
        
        nav_command.command_number = command.mission_num();


        nav_command.command_location.latitude = command.gps_location().latitude();
        nav_command.command_location.longitude = command.gps_location().longitude();
        nav_command.command_location.altitude = command.gps_location().altitude();
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
  gps->set_latitude(987.6);
  //gps->set_latitude(navSatFix->latitude);
  gps->set_longitude(123.4);
  //gps->set_longitude(navSatFix->longitude);
  gps->set_altitude(navSatFix->altitude);
}

void flightControlCallback(const monarc_uart_driver::FlightControl::ConstPtr& flightControl) {
  monarcpb::NavCPUToSysCtrl_FlightControl* flightControlProto = nav_cpu_state.mutable_control();
  flightControlProto->set_pitch(flightControl->pitch);
  flightControlProto->set_roll(flightControl->roll);
  flightControlProto->set_yaw(flightControl->yaw);
  flightControlProto->set_throttle(flightControl->throttle);
}

struct command_line_params {
  std::string uart_port;
  int baud_rate;

  bool is_test_mode;
};

command_line_params get_params() {
  ros::NodeHandle param_handle("~");

  command_line_params params;
  param_handle.param("uart_port", params.uart_port, default_uart_port);
  param_handle.param("baud_rate", params.baud_rate, default_baud_rate);
  param_handle.param("test_mode", params.is_test_mode, false);
  return params;
}

/**
 * Test mode will continuously send a series of known bytes on the UART port.
 */
void run_test_mode(UartHandler* uart) {
  ROS_INFO("Running testing mode");
  std::string test_str = std::string(30, 'A');
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    uart->write(test_str);
    loop_rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "monarc_uart_driver");
  ros::NodeHandle nh;

  command_line_params params = get_params();

  /**
   * Initialize the uart handler with the provided port.
   */
  UartHandler uart(params.uart_port, params.baud_rate);
  if (!uart.isOpen()) {
    throw serial::PortNotOpenedException(params.uart_port.c_str());
  }
  ROS_INFO("monarc_uart_driver using UART port: %s", params.uart_port.c_str());
  ROS_INFO("monarc_uart_driver using baud rate: %d", params.baud_rate);

  /**
   * Check if in test mode.
   */
  if (params.is_test_mode) {
    run_test_mode(&uart);
    return 0;
  }

  /*
   * Launch UART reader thread.
   */
  boost::thread reader_thread(uart_reader, &uart);

  /*
   * Subscribe to all necessary topics, each with a queue_size of 1
   */
  ros::Subscriber gpsSub  = nh.subscribe("fix", 1, gpsFixCallback);
  ros::Subscriber ctrlSub = nh.subscribe("flight_control", 10, flightControlCallback);

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
    nav_cpu_state.Clear();
    uart.write(state_data);

    /*
     * Sleep for the remainder of the interval.
     */
    loop_rate.sleep();
  }

  reader_thread.join();
  return 0;
}
