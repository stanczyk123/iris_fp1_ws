// IRIS FP1 - Master Node (Tahap 1b: Odometry dari kecepatan)
//
// Simulasi mengirim vel_x dan vel_y dalam cm/s pada koordinat DUNIA
// (terbukti: saat mobil berputar, vektornya ikut berputar sementara
// besarnya tetap). Jadi odometry bisa dihitung tanpa konstanta:
//
//   heading = atan2(vel_y, vel_x)
//   x += vel_x * dt
//   y += vel_y * dt
//
// Ini lebih tepat daripada rumus differential drive, karena mobil ini
// memakai Ackermann steering: belokan datang dari roda depan, bukan
// dari selisih putaran roda kiri-kanan.
//
// Encoder tetap dibaca untuk jarak tempuh total.

#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

using namespace std::chrono_literals;

class MasterNode : public rclcpp::Node
{
public:
  MasterNode() : Node("master_node"),
                 x_(0.0), y_(0.0), theta_(0.0),
                 prev_left_(0), prev_right_(0),
                 total_ticks_(0), packet_count_(0),
                 first_packet_(true)
  {
    meter_per_tick_ = this->declare_parameter<double>("meter_per_tick", 0.0024);

    // Di bawah ambang ini vektor kecepatan terlalu kecil untuk
    // menentukan arah; heading terakhir dipertahankan.
    min_speed_cms_ = this->declare_parameter<double>("min_speed_cms", 5.0);

    sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/car_feedback", 10,
      std::bind(&MasterNode::feedbackCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/odom_2d", 10);

    report_timer_ = this->create_wall_timer(
      1s, std::bind(&MasterNode::report, this));

    RCLCPP_INFO(this->get_logger(), "Master node aktif (odometry dari kecepatan).");
    RCLCPP_INFO(this->get_logger(), "Menunggu data encoder...");
  }

private:
  void feedbackCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) return;

    int32_t left  = msg->data[0];
    int32_t right = msg->data[1];
    vel_x_ = msg->data[2];
    vel_y_ = msg->data[3];

    rclcpp::Time now = this->now();

    if (first_packet_) {
      prev_left_  = left;
      prev_right_ = right;
      last_time_  = now;
      first_packet_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "Paket pertama: enc kiri=%d kanan=%d. Odometry mulai dari (0,0).",
                  left, right);
      return;
    }

    double dt = (now - last_time_).seconds();
    last_time_ = now;

    // Lewati dt yang tidak masuk akal (lonjakan jadwal, node baru bangun)
    if (dt <= 0.0 || dt > 0.5) return;

    // --- Posisi: integrasi kecepatan (cm/s -> m/s) ---
    double vx = vel_x_ / 100.0;
    double vy = vel_y_ / 100.0;

    x_ += vx * dt;
    y_ += vy * dt;

    // --- Heading: arah vektor kecepatan ---
    double speed_cms = std::hypot(static_cast<double>(vel_x_),
                                  static_cast<double>(vel_y_));
    if (speed_cms >= min_speed_cms_) {
      theta_ = std::atan2(vy, vx);
    }
    // Kalau mobil hampir diam, pertahankan heading terakhir

    // --- Jarak tempuh dari encoder ---
    total_ticks_ += ((left - prev_left_) + (right - prev_right_)) / 2;
    prev_left_  = left;
    prev_right_ = right;

    packet_count_++;

    auto pose = geometry_msgs::msg::Pose2D();
    pose.x = x_;
    pose.y = y_;
    pose.theta = theta_;
    pub_->publish(pose);
  }

  void report()
  {
    if (first_packet_) return;

    double speed_cms = std::hypot(static_cast<double>(vel_x_),
                                  static_cast<double>(vel_y_));

    RCLCPP_INFO(this->get_logger(),
                "pos=(%.2f, %.2f) m | heading=%.1f deg | speed=%.0f cm/s | "
                "jarak_enc=%.2f m | %lu paket/s",
                x_, y_, theta_ * 180.0 / M_PI, speed_cms,
                total_ticks_ * meter_per_tick_, packet_count_);

    packet_count_ = 0;
  }

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  double meter_per_tick_;
  double min_speed_cms_;

  double x_, y_, theta_;
  int32_t prev_left_, prev_right_;
  int64_t total_ticks_;
  unsigned long packet_count_;
  bool first_packet_;
  rclcpp::Time last_time_;

  int32_t vel_x_ = 0, vel_y_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MasterNode>());
  rclcpp::shutdown();
  return 0;
}