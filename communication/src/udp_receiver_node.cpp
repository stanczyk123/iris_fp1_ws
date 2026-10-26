// IRIS FP1 - UDP Receiver Node
// Objektif: "UDP packet (receive)" - 15%
//
// Bind port UDP, terima paket feedback dari simulasi (20 byte),
// publish ke /car_feedback sebagai Int32MultiArray:
//   data[0]=enc kiri, data[1]=enc kanan, data[2]=vx, data[3]=vy

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

using namespace std::chrono_literals;

#pragma pack(push, 1)
struct __attribute__((packed)) FeedbackPacket
{
  char    header[4];
  int32_t left_encoder;
  int32_t right_encoder;
  int32_t x_velocity;
  int32_t y_velocity;
};
#pragma pack(pop)

static_assert(sizeof(FeedbackPacket) == 20,
              "FeedbackPacket harus 20 byte - cek padding!");

class UdpReceiverNode : public rclcpp::Node
{
public:
  UdpReceiverNode() : Node("udp_receiver_node"), sock_fd_(-1), packet_count_(0)
  {
    listen_ip_   = this->declare_parameter<std::string>("listen_ip", "0.0.0.0");
    listen_port_ = this->declare_parameter<int>("listen_port", 8889);

    if (!setupSocket()) {
      RCLCPP_FATAL(this->get_logger(), "Gagal setup socket UDP.");
      rclcpp::shutdown();
      return;
    }

    pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
      "/car_feedback", 10);

    poll_timer_ = this->create_wall_timer(
      1ms, std::bind(&UdpReceiverNode::pollSocket, this));

    RCLCPP_INFO(this->get_logger(), "UDP receiver aktif, listen di %s:%d",
                listen_ip_.c_str(), listen_port_);
    RCLCPP_INFO(this->get_logger(), "Menunggu paket pertama...");
  }

  ~UdpReceiverNode()
  {
    if (sock_fd_ >= 0) {
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

    int reuse = 1;
    setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(listen_port_));

    if (listen_ip_ == "0.0.0.0") {
      addr.sin_addr.s_addr = INADDR_ANY;
    } else if (inet_pton(AF_INET, listen_ip_.c_str(), &addr.sin_addr) <= 0) {
      RCLCPP_ERROR(this->get_logger(), "IP tidak valid: %s", listen_ip_.c_str());
      return false;
    }

    if (bind(sock_fd_, reinterpret_cast<struct sockaddr *>(&addr),
             sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "bind() gagal di port %d: %s",
                   listen_port_, strerror(errno));
      return false;
    }

    int flags = fcntl(sock_fd_, F_GETFL, 0);
    fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);

    return true;
  }

  void pollSocket()
  {
    uint8_t buffer[256];

    while (true) {
      ssize_t len = recvfrom(sock_fd_, buffer, sizeof(buffer), 0, nullptr, nullptr);

      if (len < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "recvfrom() gagal: %s", strerror(errno));
        }
        break;
      }

      handlePacket(buffer, len);
    }
  }

  void handlePacket(const uint8_t * buffer, ssize_t len)
  {
    if (len != static_cast<ssize_t>(sizeof(FeedbackPacket))) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Ukuran paket tidak sesuai: %zd byte", len);
      return;
    }

    FeedbackPacket pkt;
    std::memcpy(&pkt, buffer, sizeof(pkt));

    if (std::memcmp(pkt.header, "IRIS", 4) != 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Header tidak valid, paket diabaikan.");
      return;
    }

    if (packet_count_ == 0) {
      RCLCPP_INFO(this->get_logger(), "Paket pertama diterima!");
    }
    packet_count_++;

    auto msg = std_msgs::msg::Int32MultiArray();
    msg.data = {pkt.left_encoder, pkt.right_encoder,
                pkt.x_velocity,   pkt.y_velocity};
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;

  int sock_fd_;
  std::string listen_ip_;
  int listen_port_;
  unsigned long packet_count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpReceiverNode>());
  rclcpp::shutdown();
  return 0;
}
