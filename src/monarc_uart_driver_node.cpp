#include <sstream>
#include <string>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "uart_handler.h"
#include "api.pb.h"

void uart_reader(UartHandler* uart) {
  ros::NodeHandle nh;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node. advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish(). Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages. If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("testing", 1000);

  while (ros::ok()) {
    std::string data = uart->read();
    if (data.length() == 0) {
      continue;
    }

    monarcpb::SysCtrlToNavCPU message;
    if (!message.ParseFromString(data)) {
      continue;
    }
    ROS_INFO("Received from UART: %s", message.DebugString().c_str());

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world! " << " " << uart->isOpen() << "...";
    message.SerializeToOstream(&ss);
    msg.data = ss.str();

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
  }
}

monarcpb::NavCPUToSysCtrl nav_cpu_state;

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it. The third argument to init() is the name of the node.
   */
  ros::init(argc, argv, "monarc_uart_driver");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh("~");

  /**
   * Initialize the uart handler with the provided port.
   */
  std::string port;
  nh.getParam("uart_port", port);
  ROS_INFO("%s", port.c_str());

  UartHandler uart(port);
  if (!uart.isOpen()) {
    throw serial::PortNotOpenedException(port.c_str());
  }

  boost::thread reader_thread(uart_reader, &uart);

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
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
