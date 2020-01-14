#include <velodyne_driver/input.h>
#include <pcap.h>

#ifndef VELODYNE_DRIVER_INPUT_PCAP_H
#define VELODYNE_DRIVER_INPUT_PCAP_H

namespace velodyne_driver
{

/** @brief Velodyne input from PCAP dump file.
 *
 * Dump files can be grabbed by libpcap, Velodyne's DSR software,
 * ethereal, wireshark, tcpdump, or the \ref vdump_command.
 */
class InputPCAP: public Input
{
public:
  InputPCAP(ros::NodeHandle private_nh,
            std::unique_ptr<TimeTranslator> time_translator,
            uint16_t port = DATA_PORT_NUMBER,
            double packet_rate = 0.0,
            std::string filename = "",
            bool read_once = false,
            bool read_fast = false,
            double repeat_delay = 0.0);
  virtual ~InputPCAP();

  int getPacket(velodyne_msgs::VelodynePacket *pkt,
                const ros::Duration time_offset) override;
  void setDeviceIP(const std::string& ip);

private:
  ros::Rate packet_rate_;
  std::string filename_;
  pcap_t *pcap_;
  bpf_program pcap_packet_filter_;
  char errbuf_[PCAP_ERRBUF_SIZE];
  bool empty_;
  bool read_once_;
  bool read_fast_;
  double repeat_delay_;
};

}  // namespace velodyne_driver

#endif  // VELODYNE_DRIVER_INPUT_PCAP_H
