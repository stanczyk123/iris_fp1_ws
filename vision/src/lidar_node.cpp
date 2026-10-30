// IRIS FP1 - LiDAR Obstacle Detection Node
// Objektif opsional: Obstacle Avoidance
//
// Membagi scan menjadi tiga sektor dan melaporkan jarak terdekat di
// masing-masing sektor:
//
//        kiri        depan        kanan
//     \    |    /     |     \     |    /
//      \   |   /      |      \    |   /
//       \  |  /    [ mobil ]   \  |  /
//
// Sektor DEPAN menjawab "apakah ada halangan di jalur saya".
// Sektor KIRI dan KANAN menjawab "sisi mana yang kosong untuk menghindar".
// Tanpa informasi sisi, mobil hanya bisa berhenti, tidak bisa memilih arah.
//
// Publish:
//   /obstacle_distance  std_msgs/Float32            jarak sektor depan
//   /obstacle_sectors   std_msgs/Float32MultiArray  [depan, kiri, kanan, sudut]
//
// Topic pertama dipertahankan agar lapisan pengaman di master tetap bekerja
// walaupun FSM penghindaran dimatikan.
//
// Jalankan:
//   ros2 run vision lidar_node
//   ros2 run vision lidar_node --ros-args -p fov_deg:=30.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

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

    // Lebar koridor di depan mobil, dalam meter. Sedikit lebih lebar
    // daripada badan mobil supaya ada margin.
    lebar_koridor_ = this->declare_parameter<double>("lebar_koridor", 2.6);

    // Seberapa jauh ke samping masih dianggap "lajur sebelah" saat menilai
    // apakah sisi itu kosong untuk menghindar.
    lebar_samping_ = this->declare_parameter<double>("lebar_samping", 4.0);

    // Jarak aman antara tepi koridor dan awal sektor samping. Tanpa celah
    // ini, halangan yang menumpang di tepi koridor ikut terbaca di sektor
    // samping, sehingga sisi itu dikira tertutup padahal justru ke sanalah
    // mobil harus menyingkir.
    margin_samping_ = this->declare_parameter<double>("margin_samping", 0.9);

    // Jarak antar as roda, dipakai menghitung jari-jari belok dari sudut setir
    wheelbase_ = this->declare_parameter<double>("wheelbase", 2.6);
    // Perbandingan antara angka steering yang dikirim ke simulasi dengan
    // sudut roda yang sebenarnya. Perbesar bila koridor kurang melengkung,
    // perkecil bila terlalu melengkung.
    faktor_setir_ = this->declare_parameter<double>("faktor_setir", 0.45);

    // true  : sudut positif pada scan berarti sisi KIRI mobil (konvensi ROS)
    // false : sebaliknya
    // Verifikasi dengan menaruh kotak di satu sisi lalu melihat sektor mana
    // yang angkanya turun. Kalau tertukar, ubah parameter ini.
    sudut_positif_kiri_ = this->declare_parameter<bool>("sudut_positif_kiri", true);

    jarak_min_valid_ = this->declare_parameter<double>("jarak_min_valid", 0.15);
    jarak_max_valid_ = this->declare_parameter<double>("jarak_max_valid", 30.0);

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarNode::scanCallback, this, std::placeholders::_1));

    pub_depan_ = this->create_publisher<std_msgs::msg::Float32>(
      "/obstacle_distance", 10);
    pub_sektor_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/obstacle_sectors", 10);

    // Sudut setir yang sedang dipakai menentukan lengkungan jalur yang akan
    // ditempuh mobil. Tanpa itu, koridor lurus akan menyapu lajur sebelah
    // setiap kali mobil berada di tikungan.
    sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/velocity_command", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        setir_deg_ = msg->angular.z;
      });

    report_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&LidarNode::report, this));

    RCLCPP_INFO(this->get_logger(), "LiDAR node aktif.");
    RCLCPP_INFO(this->get_logger(), "  subscribe : %s", scan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  publish   : /obstacle_distance, /obstacle_sectors");
    RCLCPP_INFO(this->get_logger(),
                "  koridor depan %.1f m, samping s/d %.1f m dari tepi koridor",
                lebar_koridor_, lebar_samping_);
    RCLCPP_INFO(this->get_logger(), "Menunggu scan pertama...");
  }

private:
  // Jari-jari belok dari sudut setir, model sepeda sederhana.
  // Tak hingga berarti mobil sedang lurus.
  double jariJariBelok() const
  {
    double roda_deg = std::clamp(setir_deg_ * faktor_setir_, -35.0, 35.0);
    if (std::fabs(roda_deg) < 1.0) return std::numeric_limits<double>::infinity();
    return wheelbase_ / std::tan(roda_deg * M_PI / 180.0);
  }

  // Jarak sebuah titik dari JALUR yang akan ditempuh mobil, bukan dari garis
  // lurus di depannya. Positif berarti titik itu ada di sisi sudut positif.
  //
  // Jalur mobil adalah busur lingkaran berjari-jari R yang pusatnya berada
  // di samping mobil. Simpangan sebuah titik dari busur itu adalah selisih
  // antara jaraknya ke pusat dengan R. Saat R menuju tak hingga, rumus ini
  // kembali menjadi jarak menyamping biasa.
  double sampingEfektif(double maju, double samping, double R) const
  {
    if (!std::isfinite(R) || std::fabs(R) > 1.0e4) return samping;
    double d = std::hypot(maju, samping + R) - std::fabs(R);
    return (R > 0.0) ? d : -d;
  }

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

    // Tiap berkas diubah ke koordinat mobil: seberapa jauh ke DEPAN dan
    // seberapa jauh ke SAMPING. Dengan begitu daerah yang diperiksa adalah
    // koridor selebar badan mobil, bukan kerucut sudut.
    //
    // Kerucut sudut menyempit saat jarak mengecil: pada 10 m ia mencakup
    // 2,7 m ke samping, tapi pada 3 m hanya 0,8 m. Halangan di tepi lajur
    // justru hilang dari pandangan tepat sebelum tertabrak.
    const double half_kor = lebar_koridor_ / 2.0;
    const double batas_samping = half_kor + lebar_samping_;

    double depan = std::numeric_limits<double>::infinity();
    double sisi_a = std::numeric_limits<double>::infinity();  // sisi sudut positif
    double sisi_b = std::numeric_limits<double>::infinity();  // sisi sudut negatif
    double sudut_depan = 0.0;
    double samping_depan = 0.0;

    const double R = jariJariBelok();
    radius_ = R;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double sudut = normalize(msg->angle_min + i * msg->angle_increment);
      double r = msg->ranges[i];

      if (!std::isfinite(r)) continue;
      if (r < jarak_min_valid_ || r > jarak_max_valid_) continue;
      if (r < msg->range_min || r > msg->range_max) continue;

      double maju    = r * std::cos(sudut);
      double samping = r * std::sin(sudut);

      if (maju <= 0.0) continue;   // abaikan yang di belakang

      // Diukur dari jalur melengkung yang akan ditempuh, bukan dari
      // garis lurus di depan mobil
      samping = sampingEfektif(maju, samping, R);
      double abs_samping = std::fabs(samping);

      if (abs_samping <= half_kor) {
        // Jarak yang dilaporkan adalah jarak ke DEPAN, bukan jarak lurus
        // ke titik. Itu yang menentukan kapan mobil harus mengerem.
        if (maju < depan) {
          depan = maju;
          sudut_depan = sudut * 180.0 / M_PI;
          samping_depan = samping;   // di sisi mana halangan itu berada
        }
      } else if (abs_samping >= half_kor + margin_samping_ &&
                 abs_samping <= batas_samping) {
        if (samping > 0.0) { if (maju < sisi_a) sisi_a = maju; }
        else               { if (maju < sisi_b) sisi_b = maju; }
      }
    }

    // Tidak ada halangan -> kirim angka besar, bukan infinity, supaya node
    // penerima tidak perlu menangani kasus khusus.
    auto bersihkan = [this](double v) {
      return std::isfinite(v) ? v : jarak_max_valid_;
    };

    jarak_depan_ = bersihkan(depan);
    double a = bersihkan(sisi_a);
    double b = bersihkan(sisi_b);

    jarak_kiri_  = sudut_positif_kiri_ ? a : b;
    jarak_kanan_ = sudut_positif_kiri_ ? b : a;
    sudut_depan_ = sudut_depan;

    // Dinyatakan dengan kesepakatan "positif berarti sisi kiri mobil",
    // sama seperti penamaan sektor di atas.
    samping_depan_ = sudut_positif_kiri_ ? samping_depan : -samping_depan;

    auto out = std_msgs::msg::Float32();
    out.data = static_cast<float>(jarak_depan_);
    pub_depan_->publish(out);

    auto sektor = std_msgs::msg::Float32MultiArray();
    sektor.data = {static_cast<float>(jarak_depan_),
                   static_cast<float>(jarak_kiri_),
                   static_cast<float>(jarak_kanan_),
                   static_cast<float>(samping_depan_)};
    pub_sektor_->publish(sektor);
  }

  void report()
  {
    if (scan_count_ == 0) return;

    char lengkung[32];
    if (std::isfinite(radius_) && std::fabs(radius_) < 1.0e4) {
      std::snprintf(lengkung, sizeof(lengkung), "R=%.0f m", radius_);
    } else {
      std::snprintf(lengkung, sizeof(lengkung), "lurus");
    }

    RCLCPP_INFO(this->get_logger(),
                "depan: %5.2f m (samping %+.2f m) | kiri: %5.2f m | "
                "kanan: %5.2f m | %s",
                jarak_depan_, samping_depan_, jarak_kiri_, jarak_kanan_,
                lengkung);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_depan_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_sektor_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;

  std::string scan_topic_;
  double lebar_koridor_, lebar_samping_, margin_samping_;
  double wheelbase_, faktor_setir_;
  double setir_deg_ = 0.0;
  double radius_ = std::numeric_limits<double>::infinity();
  double jarak_min_valid_, jarak_max_valid_;
  bool   sudut_positif_kiri_;

  double jarak_depan_ = 0.0, jarak_kiri_ = 0.0, jarak_kanan_ = 0.0;
  double sudut_depan_ = 0.0;
  double samping_depan_ = 0.0;   // positif = halangan ada di sisi kiri
  unsigned long scan_count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}