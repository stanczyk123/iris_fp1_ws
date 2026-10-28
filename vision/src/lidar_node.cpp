// IRIS FP1 - LiDAR Obstacle Detection Node
// Objektif opsional: Obstacle Avoidance - 10%
//
// Subscribe LaserScan dari simulasi, ambil hanya sektor DEPAN mobil,
// cari jarak terdekat di sektor itu, lalu publish ke /obstacle_distance.
//
// Keputusan berhenti/berbelok TIDAK dilakukan di sini. Node ini hanya
// mengubah data sensor mentah menjadi satu angka yang berarti, sesuai
// pembagian package: vision memproses sensor, master mengambil keputusan.
//
// Sudut dihitung dari angle_min + i * angle_increment lalu dinormalisasi,
// jadi node ini bekerja baik untuk sensor yang memakai konvensi 0..2pi
// maupun -pi..pi.
//
// Jalankan:
//   ros2 run vision lidar_node
//   ros2 run vision lidar_node --ros-args -p fov_deg:=40.0

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>

static double normalize(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

class LidarNode : public rclcpp::Node
{
public:
  LidarNode() : Node("lidar_node"), scan_count_(0)
  {
    scan_topic_ = this->declare_parameter<std::string>(
      "scan_topic", "/catvehicle/front_laser_points");

    // Lebar sektor depan yang diperiksa, dalam derajat.
    // 40 berarti -20..+20 dari arah hadap mobil.
    fov_deg_ = this->declare_parameter<double>("fov_deg", 40.0);

    // Bacaan di luar rentang ini dianggap tidak valid
    jarak_min_valid_ = this->declare_parameter<double>("jarak_min_valid", 0.15);
    jarak_max_valid_ = this->declare_parameter<double>("jarak_max_valid", 30.0);

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarNode::scanCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/obstacle_distance", 10);

    report_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&LidarNode::report, this));

    RCLCPP_INFO(this->get_logger(), "LiDAR node aktif.");
    RCLCPP_INFO(this->get_logger(), "  subscribe : %s", scan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  publish   : /obstacle_distance");
    RCLCPP_INFO(this->get_logger(), "  sektor depan: %.0f derajat", fov_deg_);
    RCLCPP_INFO(this->get_logger(), "Menunggu scan pertama...");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (scan_count_ == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "Scan pertama diterima. %zu titik, sudut %.1f..%.1f deg, "
                  "jangkauan %.2f..%.2f m",
                  msg->ranges.size(),
                  msg->angle_min * 180.0 / M_PI,
                  msg->angle_max * 180.0 / M_PI,
                  msg->range_min, msg->range_max);
    }
    scan_count_++;

    double half_fov = fov_deg_ * M_PI / 180.0 / 2.0;
    double terdekat = std::numeric_limits<double>::infinity();
    sudut_terdekat_ = 0.0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double sudut = normalize(msg->angle_min + i * msg->angle_increment);

      // Hanya perhatikan yang ada di depan mobil
      if (std::fabs(sudut) > half_fov) continue;

      double r = msg->ranges[i];

      // Buang inf, nan, dan bacaan di luar rentang wajar
      if (!std::isfinite(r)) continue;
      if (r < jarak_min_valid_ || r > jarak_max_valid_) continue;
      if (r < msg->range_min || r > msg->range_max) continue;

      if (r < terdekat) {
        terdekat = r;
        sudut_terdekat_ = sudut * 180.0 / M_PI;
      }
    }

    // Tidak ada halangan terdeteksi -> kirim angka besar, bukan infinity,
    // supaya node penerima tidak perlu menangani kasus khusus.
    jarak_terdekat_ = std::isfinite(terdekat)
      ? terdekat
      : static_cast<double>(jarak_max_valid_);

    auto out = std_msgs::msg::Float32();
    out.data = static_cast<float>(jarak_terdekat_);
    pub_->publish(out);
  }

  void report()
  {
    if (scan_count_ == 0) return;

    RCLCPP_INFO(this->get_logger(),
                "halangan terdekat: %.2f m pada %+.0f deg  (%lu scan/s)",
                jarak_terdekat_, sudut_terdekat_, scan_count_per_sec_);

    scan_count_per_sec_ = scan_count_ - scan_count_lalu_;
    scan_count_lalu_ = scan_count_;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  std::string scan_topic_;
  double fov_deg_, jarak_min_valid_, jarak_max_valid_;

  double jarak_terdekat_ = 0.0;
  double sudut_terdekat_ = 0.0;
  unsigned long scan_count_, scan_count_lalu_ = 0, scan_count_per_sec_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}