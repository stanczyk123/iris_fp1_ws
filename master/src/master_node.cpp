// IRIS FP1 - Master Node (Tahap 2: Odometry + Motion-to-point)
//
// Odometry dari vel_x/vel_y yang dikirim simulasi:
//   heading = atan2(vel_y, vel_x)
//   x += vel_x * dt,  y += vel_y * dt
//
// Target bisa dinyatakan dua cara:
//   target_relatif = true  -> (maju, kiri) relatif arah awal mobil  [default]
//   target_relatif = false -> koordinat dunia
//
// Contoh: 10 meter lurus ke depan
//   ros2 run master master_node --ros-args -p target_x:=10.0 -p target_y:=0.0
//
// Contoh: 8 meter maju, 4 meter ke kiri
//   ros2 run master master_node --ros-args -p target_x:=8.0 -p target_y:=4.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
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
    target_x_       = this->declare_parameter<double>("target_x", 10.0);
    target_y_       = this->declare_parameter<double>("target_y", 0.0);
    target_relatif_ = this->declare_parameter<bool>("target_relatif", true);

    steering_gain_ = this->declare_parameter<double>("steering_gain", 30.0);
    pwm_jalan_     = this->declare_parameter<int>("pwm_jalan", 90);
    pwm_min_       = this->declare_parameter<int>("pwm_min", 50);
    toleransi_     = this->declare_parameter<double>("toleransi", 0.5);
    jarak_pelan_   = this->declare_parameter<double>("jarak_pelan", 2.0);
    min_speed_cms_ = this->declare_parameter<double>("min_speed_cms", 5.0);

    sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/car_feedback", 10,
      std::bind(&MasterNode::feedbackCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/odom_2d", 10);
    cmd_pub_  = this->create_publisher<geometry_msgs::msg::Twist>("/velocity_command", 10);

    control_timer_ = this->create_wall_timer(
      50ms, std::bind(&MasterNode::control, this));

    report_timer_ = this->create_wall_timer(
      1s, std::bind(&MasterNode::report, this));

    RCLCPP_INFO(this->get_logger(), "Master node aktif (motion-to-point).");
    if (target_relatif_) {
      RCLCPP_INFO(this->get_logger(),
                  "  target relatif: maju %.1f m, kiri %.1f m",
                  target_x_, target_y_);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "  target dunia: (%.2f, %.2f)", target_x_, target_y_);
    }
    RCLCPP_INFO(this->get_logger(), "Menunggu data encoder...");
  }

private:
  // ----------------------------------------------------------------
  // Odometry
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

    double vx = vel_x_ / 100.0;
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

  // ----------------------------------------------------------------
  // Kontrol menuju target
  // ----------------------------------------------------------------
  void control()
  {
    if (first_packet_) return;

    // Mobil yang diam tidak punya arah hadap yang bisa dibaca dari
    // kecepatan. Jadi maju lurus pelan dulu sampai arahnya terbaca,
    // baru target relatif diterjemahkan ke koordinat dunia.
    if (target_relatif_ && !target_siap_) {
      double speed = std::hypot(static_cast<double>(vel_x_),
                                static_cast<double>(vel_y_));
      if (speed < 30.0) {
        kirim(70, 0);
        return;
      }

      double maju = target_x_;
      double kiri = target_y_;

      world_target_x_ = x_ + maju * std::cos(theta_) - kiri * std::sin(theta_);
      world_target_y_ = y_ + maju * std::sin(theta_) + kiri * std::cos(theta_);
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

    double sudut_target = std::atan2(dy, dx);
    error_ = normalize(sudut_target - theta_);

    double steering = std::clamp(-error_ * steering_gain_, -90.0, 90.0);

    int pwm = pwm_jalan_;

    // Pelan saat harus belok tajam, supaya tidak melebar
    double faktor_belok = 1.0 - std::min(std::fabs(error_) / (M_PI / 2.0), 0.7);
    pwm = static_cast<int>(pwm * faktor_belok);

    // Pelan saat target sudah dekat
    if (jarak_ < jarak_pelan_) {
      pwm = static_cast<int>(pwm * (jarak_ / jarak_pelan_));
    }

    pwm = std::clamp(std::max(pwm, pwm_min_), -255, 255);

    kirim(pwm, static_cast<int>(steering));
  }

  void kirim(int pwm, int steering)
  {
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

    double tx = target_relatif_ ? world_target_x_ : target_x_;
    double ty = target_relatif_ ? world_target_y_ : target_y_;

    RCLCPP_INFO(this->get_logger(),
                "pos=(%.2f, %.2f) hdg=%.0f | target=(%.1f, %.1f) "
                "jarak=%.2f m err=%.0f deg | pwm=%d steer=%d%s",
                x_, y_, theta_ * 180.0 / M_PI,
                tx, ty,
                jarak_, error_ * 180.0 / M_PI,
                last_pwm_, last_steering_,
                sampai_ ? "  [SAMPAI]" : "");
  }

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  double target_x_, target_y_;
  bool   target_relatif_;
  bool   target_siap_ = false;
  double world_target_x_ = 0.0, world_target_y_ = 0.0;

  double steering_gain_, toleransi_, jarak_pelan_, min_speed_cms_;
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