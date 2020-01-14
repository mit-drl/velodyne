#include <velodyne_driver/input.h>

#ifndef VELODYNE_DRIVER_INPUT_SOCKET_H
#define VELODYNE_DRIVER_INPUT_SOCKET_H


namespace velodyne_driver
{

/** @brief Live Velodyne input from socket. */
class InputSocket: public Input
{
public:
  InputSocket(ros::NodeHandle private_nh,
              uint16_t port = DATA_PORT_NUMBER);
  virtual ~InputSocket();

  virtual int getPacket(velodyne_msgs::VelodynePacket *pkt,
                        const double time_offset);
  void setDeviceIP(const std::string& ip);

private:
  int sockfd_;
  in_addr devip_;
};

}  // namespace velodyne_driver

#endif  // VELODYNE_DRIVER_INPUT_SOCKET_H
