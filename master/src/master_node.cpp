// IRIS FP1 - Master Node
// Objektif 3: Motion-to-point            (mode: point)
// Objektif 6: Full Autonomous Car        (mode: follow)
// Objektif opsional: Obstacle Avoidance  (lapisan pengaman, kedua mode)
//
// Odometry dari vel_x/vel_y yang dikirim simulasi:
//   heading = atan2(vel_y, vel_x)
//   x += vel_x * dt,  y += vel_y * dt
//
// Lapisan pengaman halangan diterapkan di fungsi kirim(), yaitu satu-satunya
// jalan keluar perintah. Dengan begitu mode point maupun follow sama-sama
// terlindungi tanpa logika ganda, dan tidak ada jalur yang bisa melewatinya.
//
// Catatan tanda steering: pada simulasi ini steering POSITIF = belok KANAN.
//   - Mode point : error sudut positif berarti target di KIRI, jadi dibalik.
//   - Mode follow: offset piksel positif berarti target di KANAN, tidak dibalik.
//
// Contoh:
//   ros2 run master master_node --ros-args -p mode:=point -p target_x:=10.0
//   ros2 run master master_node --ros-args -p mode:=follow

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

static double normalize(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

class MasterNode : public rclcpp::Node
{
public:
  MasterNode() : Node("master_node"),
                 x_(0.0), y_(0.0), theta_(0.0),
                 prev_left_(0), prev_right_(0),
                 total_ticks_(0), first_packet_(true),
                 sampai_(false)
  {
    mode_ = this->declare_parameter<std::string>("mode", "point");

    // --- Mode point ---
    target_x_       = this->declare_parameter<double>("target_x", 10.0);
    target_y_       = this->declare_parameter<double>("target_y", 0.0);
    target_relatif_ = this->declare_parameter<bool>("target_relatif", true);
    steering_gain_  = this->declare_parameter<double>("steering_gain", 30.0);
    toleransi_      = this->declare_parameter<double>("toleransi", 0.5);
    jarak_pelan_    = this->declare_parameter<double>("jarak_pelan", 2.0);

    // --- Mode follow ---
    lebar_gambar_      = this->declare_parameter<int>("lebar_gambar", 800);
    follow_gain_       = this->declare_parameter<double>("follow_gain", 45.0);
    target_timeout_ms_ = this->declare_parameter<int>("target_timeout_ms", 1000);

    // --- Pengaman halangan ---
    // Ambang berhenti harus jauh di atas range_min LiDAR (1.5 m), karena
    // halangan yang lebih dekat dari itu tidak terbaca sama sekali.
    obstacle_stop_       = this->declare_parameter<double>("obstacle_stop", 5.0);
    obstacle_slow_       = this->declare_parameter<double>("obstacle_slow", 10.0);
    obstacle_timeout_ms_ = this->declare_parameter<int>("obstacle_timeout_ms", 1000);

    // --- Umum ---
    pwm_jalan_     = this->declare_parameter<int>("pwm_jalan", 90);
    pwm_min_       = this->declare_parameter<int>("pwm_min", 50);
    min_speed_cms_ = this->declare_parameter<double>("min_speed_cms", 5.0);

    sub_feedback_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/car_feedback", 10,
      std::bind(&MasterNode::feedbackCallback, this, std::placeholders::_1));

    sub_target_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/turn_target", 10,
      std::bind(&MasterNode::turnTargetCallback, this, std::placeholders::_1));

    sub_obstacle_ = this->create_subscription<std_msgs::msg::Float32>(
      "/obstacle_distance", 10,
      std::bind(&MasterNode::obstacleCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/odom_2d", 10);
    cmd_pub_  = this->create_publisher<geometry_msgs::msg::Twist>("/velocity_command", 10);

    control_timer_ = this->create_wall_timer(
      50ms, std::bind(&MasterNode::control, this));

    report_timer_ = this->create_wall_timer(
      1s, std::bind(&MasterNode::report, this));

    if (mode_ == "follow") {
      RCLCPP_INFO(this->get_logger(), "Master node aktif (mode: FOLLOW).");
      RCLCPP_INFO(this->get_logger(), "  mengikuti /turn_target dari vision");
    } else {
      RCLCPP_INFO(this->get_logger(), "Master node aktif (mode: POINT).");
      if (target_relatif_) {
        RCLCPP_INFO(this->get_logger(), "  target: maju %.1f m, kiri %.1f m",
                    target_x_, target_y_);
      } else {
        RCLCPP_INFO(this->get_logger(), "  target dunia: (%.2f, %.2f)",
                    target_x_, target_y_);
      }
    }
    RCLCPP_INFO(this->get_logger(),
                "  pengaman: melambat di %.1f m, berhenti di %.1f m",
                obstacle_slow_, obstacle_stop_);
    RCLCPP_INFO(this->get_logger(), "Menunggu data encoder...");
  }

private:
  // ----------------------------------------------------------------
  // Odometry dari feedback simulasi
  // ----------------------------------------------------------------
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
      RCLCPP_INFO(this->get_logger(), "Odometry dimulai dari (0,0).");
      return;
    }

    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (dt <= 0.0 || dt > 0.5) return;

    double vx = vel_x_ / 100.0;   // cm/s -> m/s
    double vy = vel_y_ / 100.0;

    x_ += vx * dt;
    y_ += vy * dt;

    double speed = std::hypot(vx, vy) * 100.0;
    if (speed >= min_speed_cms_) {
      theta_ = std::atan2(vy, vx);
    }

    total_ticks_ += ((left - prev_left_) + (right - prev_right_)) / 2;
    prev_left_  = left;
    prev_right_ = right;

    auto pose = geometry_msgs::msg::Pose2D();
    pose.x = x_;
    pose.y = y_;
    pose.theta = theta_;
    odom_pub_->publish(pose);
  }

  void turnTargetCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    turn_target_x_ = msg->x;
    turn_valid_    = (msg->z > 0.5);
    last_target_time_ = this->now();
    ada_target_ = true;
  }

  void obstacleCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    obstacle_dist_ = msg->data;
    last_obstacle_time_ = this->now();
    ada_obstacle_ = true;
  }

  // ----------------------------------------------------------------
  // Kontrol
  // ----------------------------------------------------------------
  void control()
  {
    if (first_packet_) return;

    if (mode_ == "follow") {
      controlFollow();
    } else {
      controlPoint();
    }
  }

  void controlFollow()
  {
    double umur_ms = ada_target_
      ? (this->now() - last_target_time_).nanoseconds() / 1.0e6
      : 1.0e9;

    if (umur_ms > target_timeout_ms_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Tidak ada /turn_target. Mobil berhenti.");
      kirim(0, 0);
      return;
    }

    double tengah = lebar_gambar_ / 2.0;
    offset_ = turn_target_x_ - tengah;
    double norm = std::clamp(offset_ / tengah, -1.0, 1.0);

    double steering = std::clamp(norm * follow_gain_, -90.0, 90.0);

    int pwm = static_cast<int>(pwm_jalan_ * (1.0 - 0.6 * std::fabs(norm)));
    pwm = std::clamp(std::max(pwm, pwm_min_), -255, 255);

    kirim(pwm, static_cast<int>(steering));
  }

  void controlPoint()
  {
    if (target_relatif_ && !target_siap_) {
      double speed = std::hypot(static_cast<double>(vel_x_),
                                static_cast<double>(vel_y_));
      if (speed < 30.0) {
        kirim(70, 0);
        return;
      }

      world_target_x_ = x_ + target_x_ * std::cos(theta_) - target_y_ * std::sin(theta_);
      world_target_y_ = y_ + target_x_ * std::sin(theta_) + target_y_ * std::cos(theta_);
      target_siap_ = true;

      RCLCPP_INFO(this->get_logger(),
                  "Arah awal terbaca: %.0f deg. Target dunia = (%.2f, %.2f)",
                  theta_ * 180.0 / M_PI, world_target_x_, world_target_y_);
    }

    double tx = target_relatif_ ? world_target_x_ : target_x_;
    double ty = target_relatif_ ? world_target_y_ : target_y_;

    double dx = tx - x_;
    double dy = ty - y_;
    jarak_ = std::hypot(dx, dy);

    if (jarak_ < toleransi_) {
      if (!sampai_) {
        RCLCPP_INFO(this->get_logger(),
                    "Target tercapai! pos=(%.2f, %.2f), sisa %.2f m",
                    x_, y_, jarak_);
        sampai_ = true;
      }
      kirim(0, 0);
      return;
    }
    sampai_ = false;

    error_ = normalize(std::atan2(dy, dx) - theta_);

    double steering = std::clamp(-error_ * steering_gain_, -90.0, 90.0);

    int pwm = pwm_jalan_;

    double faktor_belok = 1.0 - std::min(std::fabs(error_) / (M_PI / 2.0), 0.7);
    pwm = static_cast<int>(pwm * faktor_belok);

    if (jarak_ < jarak_pelan_) {
      pwm = static_cast<int>(pwm * (jarak_ / jarak_pelan_));
    }

    pwm = std::clamp(std::max(pwm, pwm_min_), -255, 255);

    kirim(pwm, static_cast<int>(steering));
  }

  // ----------------------------------------------------------------
  // Satu-satunya jalan keluar perintah, sekaligus lapisan pengaman
  // ----------------------------------------------------------------
  void kirim(int pwm, int steering)
  {
    rem_aktif_ = false;

    // Pengaman hanya berlaku saat maju. Mundur dibiarkan supaya mobil
    // masih bisa lepas kalau terlanjur mepet halangan.
    if (ada_obstacle_ && pwm > 0) {
      double umur_ms = (this->now() - last_obstacle_time_).nanoseconds() / 1.0e6;

      // Data LiDAR basi diabaikan, supaya mobil tidak ikut mati kalau
      // node lidar dimatikan di tengah jalan.
      if (umur_ms < obstacle_timeout_ms_) {
        if (obstacle_dist_ < obstacle_stop_) {
          pwm = 0;
          rem_aktif_ = true;
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "HALANGAN %.2f m - berhenti.", obstacle_dist_);
        } else if (obstacle_dist_ < obstacle_slow_) {
          double f = (obstacle_dist_ - obstacle_stop_) /
                     (obstacle_slow_ - obstacle_stop_);
          pwm = static_cast<int>(pwm * f);
          rem_aktif_ = true;
        }
      }
    }

    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x  = static_cast<double>(pwm);
    cmd.angular.z = static_cast<double>(steering);
    cmd_pub_->publish(cmd);

    last_pwm_ = pwm;
    last_steering_ = steering;
  }

  void report()
  {
    if (first_packet_) return;

    char info_halangan[64];
    if (!ada_obstacle_) {
      std::snprintf(info_halangan, sizeof(info_halangan), " | lidar: -");
    } else {
      std::snprintf(info_halangan, sizeof(info_halangan), " | lidar: %.1f m%s",
                    obstacle_dist_, rem_aktif_ ? "  [REM]" : "");
    }

    if (mode_ == "follow") {
      RCLCPP_INFO(this->get_logger(),
                  "FOLLOW | pos=(%.1f, %.1f) hdg=%.0f | offset=%+.0f px %s | "
                  "pwm=%d steer=%d%s",
                  x_, y_, theta_ * 180.0 / M_PI,
                  offset_, turn_valid_ ? "[lihat]" : "[tebak]",
                  last_pwm_, last_steering_, info_halangan);
    } else {
      double tx = target_relatif_ ? world_target_x_ : target_x_;
      double ty = target_relatif_ ? world_target_y_ : target_y_;
      RCLCPP_INFO(this->get_logger(),
                  "POINT | pos=(%.2f, %.2f) hdg=%.0f | target=(%.1f, %.1f) "
                  "jarak=%.2f m | pwm=%d steer=%d%s%s",
                  x_, y_, theta_ * 180.0 / M_PI, tx, ty,
                  jarak_, last_pwm_, last_steering_,
                  sampai_ ? "  [SAMPAI]" : "", info_halangan);
    }
  }

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_feedback_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_target_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_obstacle_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  std::string mode_;

  double target_x_, target_y_;
  bool   target_relatif_;
  bool   target_siap_ = false;
  double world_target_x_ = 0.0, world_target_y_ = 0.0;
  double steering_gain_, toleransi_, jarak_pelan_, min_speed_cms_;

  int    lebar_gambar_;
  double follow_gain_;
  int    target_timeout_ms_;
  double turn_target_x_ = 0.0;
  bool   turn_valid_ = false;
  bool   ada_target_ = false;
  rclcpp::Time last_target_time_;
  double offset_ = 0.0;

  double obstacle_stop_, obstacle_slow_;
  int    obstacle_timeout_ms_;
  double obstacle_dist_ = 0.0;
  bool   ada_obstacle_ = false;
  bool   rem_aktif_ = false;
  rclcpp::Time last_obstacle_time_;

  int pwm_jalan_, pwm_min_;

  double x_, y_, theta_;
  int32_t prev_left_, prev_right_;
  int64_t total_ticks_;
  bool first_packet_;
  bool sampai_;
  rclcpp::Time last_time_;

  int32_t vel_x_ = 0, vel_y_ = 0;
  double jarak_ = 0.0, error_ = 0.0;
  int last_pwm_ = 0, last_steering_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MasterNode>());
  rclcpp::shutdown();
  return 0;
}