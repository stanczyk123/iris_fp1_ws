// IRIS FP1 - UDP Sender Node
// Objektif: "UDP packet (send)" - 15%
//
// Subscribe /velocity_command (geometry_msgs/Twist), kirim ke simulasi
// lewat UDP: header "IRIS" + int32 pwm + int32 steering = 12 byte.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

#pragma pack(push, 1)
struct __attribute__((packed)) VelocityPacket
{
  char    header[4];
  int32_t motor_pwm;
  int32_t steering;
};
#pragma pack(pop)

static_assert(sizeof(VelocityPacket) == 12,
              "VelocityPacket harus 12 byte - cek padding!");

class UdpSenderNode : public rclcpp::Node
{
public:
  UdpSenderNode() : Node("udp_sender_node"), sock_fd_(-1), has_command_(false)
  {
    sim_ip_   = this->declare_parameter<std::string>("sim_ip", "127.0.0.1");
    sim_port_ = this->declare_parameter<int>("sim_port", 8888);
    int watchdog_ms = this->declare_parameter<int>("watchdog_ms", 500);

    if (!setupSocket()) {
      RCLCPP_FATAL(this->get_logger(), "Gagal membuat socket UDP.");
      rclcpp::shutdown();
      return;
    }

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/velocity_command", 10,
      std::bind(&UdpSenderNode::commandCallback, this, std::placeholders::_1));

    last_command_time_ = this->now();
    watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(watchdog_ms / 2),
      [this, watchdog_ms]() {
        if (!has_command_) return;
        auto elapsed = (this->now() - last_command_time_).nanoseconds() / 1000000;
        if (elapsed > watchdog_ms) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "Tidak ada perintah baru, kirim stop.");
          sendPacket(0, 0);
        }
      });

    RCLCPP_INFO(this->get_logger(), "UDP sender aktif -> %s:%d",
                sim_ip_.c_str(), sim_port_);
  }

  ~UdpSenderNode()
  {
    if (sock_fd_ >= 0) {
      sendPacket(0, 0);
      close(sock_fd_);
    }
  }

private:
  bool setupSocket()
  {
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "socket() gagal: %s", strerror(errno));
      return false;
    }

    std::memset(&dest_addr_, 0, sizeof(dest_addr_));
    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port   = htons(static_cast<uint16_t>(sim_port_));

    if (inet_pton(AF_INET, sim_ip_.c_str(), &dest_addr_.sin_addr) <= 0) {
      RCLCPP_ERROR(this->get_logger(), "IP tidak valid: %s", sim_ip_.c_str());
      return false;
    }
    return true;
  }

  void commandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    int pwm      = std::clamp(static_cast<int>(msg->linear.x),  -255, 255);
    int steering = std::clamp(static_cast<int>(msg->angular.z),  -90,  90);

    last_command_time_ = this->now();
    has_command_ = true;

    sendPacket(pwm, steering);
  }

  void sendPacket(int pwm, int steering)
  {
    VelocityPacket pkt;
    std::memcpy(pkt.header, "IRIS", 4);
    pkt.motor_pwm = static_cast<int32_t>(pwm);
    pkt.steering  = static_cast<int32_t>(steering);

    ssize_t sent = sendto(sock_fd_, &pkt, sizeof(pkt), 0,
                          reinterpret_cast<struct sockaddr *>(&dest_addr_),
                          sizeof(dest_addr_));

    if (sent < 0) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "sendto() gagal: %s", strerror(errno));
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  int sock_fd_;
  struct sockaddr_in dest_addr_;
  std::string sim_ip_;
  int sim_port_;

  rclcpp::Time last_command_time_;
  bool has_command_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpSenderNode>());
  rclcpp::shutdown();
  return 0;
}
