// IRIS FP1 - Keyboard Teleop Node
// Objektif: "WASD Keyboard Movement" - 10%
//
// Membaca tombol WASD dari keyboard, lalu publish ke /velocity_command.
// Node udp_sender_node yang meneruskannya ke simulasi lewat UDP, jadi
// node ini sekaligus menguji jalur komunikasi UDP yang sudah dibuat.
//
// Kontrol:
//   W / S  : tambah / kurang PWM (S juga bisa untuk mundur)
//   A / D  : belok kiri / kanan
//   X      : luruskan setir
//   SPASI  : berhenti total
//   Q      : keluar
//
// Terminal diubah ke mode RAW supaya tombol terbaca tanpa perlu Enter,
// dan dikembalikan ke mode semula saat node dimatikan.

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

namespace
{
termios g_termios_asli;
bool    g_termios_tersimpan = false;

void kembalikanTerminal()
{
  if (g_termios_tersimpan) {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_termios_asli);
  }
}
}  // namespace

class KeyboardTeleopNode : public rclcpp::Node
{
public:
  KeyboardTeleopNode() : Node("keyboard_teleop_node"), pwm_(0), steering_(0)
  {
    pwm_step_      = this->declare_parameter<int>("pwm_step", 20);
    steering_step_ = this->declare_parameter<int>("steering_step", 10);
    pwm_max_       = this->declare_parameter<int>("pwm_max", 150);
    steering_max_  = this->declare_parameter<int>("steering_max", 60);

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/velocity_command", 10);

    setupTerminal();

    // Baca tombol sesering mungkin supaya terasa responsif
    key_timer_ = this->create_wall_timer(
      20ms, std::bind(&KeyboardTeleopNode::bacaTombol, this));

    // Kirim perintah pada laju tetap. Ini penting: udp_sender_node punya
    // watchdog yang menghentikan mobil kalau tidak ada perintah baru.
    send_timer_ = this->create_wall_timer(
      50ms, std::bind(&KeyboardTeleopNode::kirim, this));

    tampilkanBantuan();
  }

  ~KeyboardTeleopNode()
  {
    // Berhentikan mobil sebelum keluar
    auto cmd = geometry_msgs::msg::Twist();
    pub_->publish(cmd);
    kembalikanTerminal();
  }

private:
  void setupTerminal()
  {
    tcgetattr(STDIN_FILENO, &g_termios_asli);
    g_termios_tersimpan = true;
    std::atexit(kembalikanTerminal);

    termios raw = g_termios_asli;
    // ICANON mati: tombol langsung terbaca tanpa Enter
    // ECHO mati  : tombol tidak ikut tercetak di layar
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    // Non-blocking supaya timer tidak macet saat tidak ada tombol ditekan
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  }

  void tampilkanBantuan()
  {
    std::printf("\r\n");
    std::printf("=== IRIS FP1 - Kontrol Keyboard ===\r\n");
    std::printf("  W / S  : tambah / kurang PWM\r\n");
    std::printf("  A / D  : belok kiri / kanan\r\n");
    std::printf("  X      : luruskan setir\r\n");
    std::printf("  SPASI  : berhenti\r\n");
    std::printf("  Q      : keluar\r\n");
    std::printf("====================================\r\n\r\n");
    std::fflush(stdout);
  }

  void bacaTombol()
  {
    char c;
    bool berubah = false;

    // Habiskan semua tombol yang mengantre di buffer
    while (read(STDIN_FILENO, &c, 1) == 1) {
      switch (c) {
        case 'w': case 'W':
          pwm_ = std::min(pwm_ + pwm_step_, pwm_max_);
          berubah = true;
          break;
        case 's': case 'S':
          pwm_ = std::max(pwm_ - pwm_step_, -pwm_max_);
          berubah = true;
          break;
        case 'a': case 'A':
          // Steering negatif = belok kiri pada simulasi ini
          steering_ = std::max(steering_ - steering_step_, -steering_max_);
          berubah = true;
          break;
        case 'd': case 'D':
          steering_ = std::min(steering_ + steering_step_, steering_max_);
          berubah = true;
          break;
        case 'x': case 'X':
          steering_ = 0;
          berubah = true;
          break;
        case ' ':
          pwm_ = 0;
          steering_ = 0;
          berubah = true;
          break;
        case 'q': case 'Q':
          std::printf("\r\nKeluar.\r\n");
          std::fflush(stdout);
          rclcpp::shutdown();
          return;
        default:
          break;
      }
    }

    if (berubah) {
      std::printf("\rPWM: %+4d   Steering: %+3d deg     ", pwm_, steering_);
      std::fflush(stdout);
    }
  }

  void kirim()
  {
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x  = static_cast<double>(pwm_);
    cmd.angular.z = static_cast<double>(steering_);
    pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr key_timer_;
  rclcpp::TimerBase::SharedPtr send_timer_;

  int pwm_, steering_;
  int pwm_step_, steering_step_;
  int pwm_max_, steering_max_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleopNode>());
  rclcpp::shutdown();
  kembalikanTerminal();
  return 0;
}