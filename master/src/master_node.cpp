// IRIS FP1 - Master Node
// Objektif 3: Motion-to-point            (mode: point)
// Objektif 6: Full Autonomous Car        (mode: follow)
// Objektif opsional: Obstacle Avoidance  (FSM penghindaran + pengaman)
//
// Odometry dari vel_x/vel_y yang dikirim simulasi:
//   heading = atan2(vel_y, vel_x)
//   x += vel_x * dt,  y += vel_y * dt
//
// ------------------------------------------------------------------
// FINITE STATE MACHINE penghindaran halangan (hanya pada mode follow)
// ------------------------------------------------------------------
//
//                    halangan dekat & ada sisi kosong
//        IKUTI_LAJUR ---------------------------------> MENGHINDAR
//             ^                                              |
//             |                                 jalur depan bersih
//             |                                              v
//             +------------------- KEMBALI <-----------------+
//                 offset habis       |
//                                    | halangan muncul lagi
//                                    v
//                                MENGHINDAR
//
//        dari keadaan mana pun, bila halangan sangat dekat DAN
//        tidak ada sisi yang kosong:            -> BERHENTI
//
// Penghindaran dilakukan dengan menggeser TARGET ke samping, bukan dengan
// memaksa sudut setir. Jadi mobil tetap mengikuti lajur sambil bergeser,
// dan kemudi tetap ditangani oleh satu jalur perhitungan yang sama.
// Geseran dinaikkan bertahap (ramp), bukan seketika, supaya setir tidak
// menyentak dan mobil tidak kehilangan lajur.
//
// Catatan tanda steering: pada simulasi ini steering POSITIF = belok KANAN.
//   - Mode point : error sudut positif berarti target di KIRI, jadi dibalik.
//   - Mode follow: offset piksel positif berarti target di KANAN, tidak dibalik.
//
// Contoh:
//   ros2 run master master_node --ros-args -p mode:=follow
//   ros2 run master master_node --ros-args -p mode:=follow -p hindar_aktif:=false
//   ros2 run master master_node --ros-args -p mode:=point -p target_x:=10.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
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

enum class Keadaan { IKUTI_LAJUR, MENGHINDAR, KEMBALI, BERHENTI, KOREKSI };

static const char * namaKeadaan(Keadaan k)
{
  switch (k) {
    case Keadaan::IKUTI_LAJUR: return "IKUTI";
    case Keadaan::MENGHINDAR:  return "HINDAR";
    case Keadaan::KEMBALI:     return "KEMBALI";
    case Keadaan::BERHENTI:    return "BERHENTI";
    case Keadaan::KOREKSI:     return "KOREKSI";
  }
  return "?";
}

class MasterNode : public rclcpp::Node
{
public:
  MasterNode() : Node("master_node"),
                 x_(0.0), y_(0.0), theta_(0.0),
                 prev_left_(0), prev_right_(0),
                 first_packet_(true), sampai_(false)
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
    obstacle_stop_       = this->declare_parameter<double>("obstacle_stop", 5.0);
    // Harus LEBIH DEKAT daripada jarak_hindar. Kalau sama, mobil mulai
    // melambat pada jarak yang sama dengan saat FSM memutuskan menyingkir,
    // sehingga ia merayap mendekat tanpa pernah sempat bermanuver.
    obstacle_slow_       = this->declare_parameter<double>("obstacle_slow", 8.0);
    obstacle_timeout_ms_ = this->declare_parameter<int>("obstacle_timeout_ms", 1000);

    // --- FSM penghindaran ---
    hindar_aktif_   = this->declare_parameter<bool>("hindar_aktif", true);
    // Jarak saat mobil mulai bergeser ke samping. Harus lebih jauh daripada
    // obstacle_stop supaya sempat menghindar sebelum pengereman bekerja.
    // Cukup jauh agar manuver sempat selesai sebelum mencapai halangan,
    // tapi tidak terlalu jauh sampai mobil bereaksi pada hal yang belum
    // relevan. Osilasi dicegah oleh histeresis, bukan oleh jarak kecil.
    jarak_hindar_   = this->declare_parameter<double>("jarak_hindar", 15.0);
    // Ruang kosong minimum di satu sisi agar sisi itu dianggap bisa dilewati
    jarak_bebas_    = this->declare_parameter<double>("jarak_bebas", 7.0);
    // Seberapa jauh target digeser ke samping, dalam piksel BEV.
    // Cukup untuk melewati halangan selebar satu meter tanpa keluar jalan.
    // Nilai sebesar satu lajur penuh membuat setir langsung mentok dan
    // mobil terlempar keluar lintasan.
    // Satu lajur penuh. Karena mobil hanya boleh menyeberangi garis
    // putus-putus, geserannya memang harus sebesar perpindahan antar lajur,
    // bukan sekadar menepi.
    offset_hindar_  = this->declare_parameter<double>("offset_hindar", 300.0);
    // Dorongan setir langsung saat menyingkir, dalam derajat
    bias_hindar_    = this->declare_parameter<double>("bias_hindar", 18.0);
    // Laju perubahan geseran, piksel per detik
    laju_ramp_      = this->declare_parameter<double>("laju_ramp", 500.0);
    // Jarak yang harus ditempuh setelah jalur bersih sebelum kembali ke lajur
    jarak_lewat_    = this->declare_parameter<double>("jarak_lewat", 6.0);
    // Lama minimum bertahan di satu keadaan sebelum boleh berpindah lagi.
    // Tanpa ini FSM bergetar bolak-balik di sekitar ambang jarak.
    dwell_min_      = this->declare_parameter<double>("dwell_min", 1.5);
    // Ambang pengereman saat sedang menghindar. Dibuat lebih kecil supaya
    // mobil tidak berhenti padahal sedang dalam proses menyingkir.
    stop_saat_hindar_ = this->declare_parameter<double>("stop_saat_hindar", 1.8);
    // Lama berdiam di BERHENTI sebelum mobil mencoba mundur untuk
    // mendapatkan ruang bermanuver.
    sabar_berhenti_   = this->declare_parameter<double>("sabar_berhenti", 2.5);
    pwm_mundur_       = this->declare_parameter<int>("pwm_mundur", -60);
    // true: garis solid dianggap tepi jalan dan tidak boleh dilintasi saat
    // menghindar. Matikan hanya untuk menguji perilaku tanpa aturan ini.
    hormati_garis_  = this->declare_parameter<bool>("hormati_garis", true);
    // Susunan garis pada lajur yang BENAR. true berarti garis putus-putus
    // ada di kiri dan garis solid di kanan. Dipakai untuk mengenali apakah
    // mobil masih berada di lajur seberang setelah menghindar.
    putus_di_kiri_  = this->declare_parameter<bool>("putus_di_kiri", true);
    // Jarak tempuh maksimum untuk mendorong mobil kembali ke lajur asal,
    // dihitung setelah fase menahan selesai.
    batas_pulang_   = this->declare_parameter<double>("batas_pulang", 20.0);
    // Batas aman berapa jauh geseran boleh ditahan sambil menunggu halangan
    // terlewati, agar mobil tidak menahan selamanya bila sektor sampingnya
    // kebetulan terganggu sesuatu yang lain.
    batas_tahan_    = this->declare_parameter<double>("batas_tahan", 25.0);
    // Jarak aman yang dijaga dari garis solid, dalam piksel BEV
    margin_garis_   = this->declare_parameter<double>("margin_garis", 70.0);
    // Jarak mobil ke garis solid yang sudah dianggap melanggar, piksel BEV.
    // Setengah lebar lajur sekitar 190 piksel, jadi 55 berarti mobil sudah
    // berada sangat dekat dengan tepi jalan.
    ambang_garis_   = this->declare_parameter<double>("ambang_garis", 55.0);
    // Jarak mundur maksimum saat menata ulang
    jarak_koreksi_  = this->declare_parameter<double>("jarak_koreksi", 4.0);
    // Ambang menghindar lagi saat masih dalam perjalanan pulang ke lajur
    jarak_hindar_lagi_ = this->declare_parameter<double>("jarak_hindar_lagi", 9.0);

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

    sub_sektor_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/obstacle_sectors", 10,
      std::bind(&MasterNode::sektorCallback, this, std::placeholders::_1));

    sub_lane_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/lane_info", 10,
      std::bind(&MasterNode::laneCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/odom_2d", 10);
    cmd_pub_  = this->create_publisher<geometry_msgs::msg::Twist>("/velocity_command", 10);

    control_timer_ = this->create_wall_timer(
      50ms, std::bind(&MasterNode::control, this));

    report_timer_ = this->create_wall_timer(
      1s, std::bind(&MasterNode::report, this));

    if (mode_ == "follow") {
      RCLCPP_INFO(this->get_logger(), "Master node aktif (mode: FOLLOW).");
      RCLCPP_INFO(this->get_logger(), "  mengikuti /turn_target dari vision");
      RCLCPP_INFO(this->get_logger(), "  penghindaran: %s",
                  hindar_aktif_ ? "AKTIF (FSM)" : "mati, hanya berhenti");
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
                "  halangan: geser di %.1f m, melambat di %.1f m, berhenti di %.1f m",
                jarak_hindar_, obstacle_slow_, obstacle_stop_);
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

    double vx = vel_x_ / 100.0;   // cm/s -> m/s
    double vy = vel_y_ / 100.0;

    x_ += vx * dt;
    y_ += vy * dt;

    double laju = std::hypot(vx, vy);
    jarak_tempuh_ += laju * dt;   // dipakai FSM untuk mengukur "sudah lewat"

    if (laju * 100.0 >= min_speed_cms_) {
      theta_ = std::atan2(vy, vx);
    }

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

  void sektorCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) return;
    sek_depan_ = msg->data[0];
    sek_kiri_  = msg->data[1];
    sek_kanan_ = msg->data[2];
    if (msg->data.size() >= 4) samping_depan_ = msg->data[3];
    ada_sektor_ = true;
  }

  // Jenis garis pembatas dari vision: 0 = tidak terdeteksi, 1 = putus, 2 = solid
  void laneCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) return;
    jenis_kiri_  = static_cast<int>(msg->data[2]);
    jenis_kanan_ = static_cast<int>(msg->data[3]);
    if (msg->data.size() >= 7) {
      x_kiri_  = msg->data[5];
      x_kanan_ = msg->data[6];
    }
    ada_lane_ = true;
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

  // Perpindahan keadaan dicatat waktunya, agar bisa dipaksa bertahan
  // sejenak sebelum boleh berpindah lagi.
  void setKeadaan(Keadaan k)
  {
    if (k == keadaan_) return;
    keadaan_ = k;
    masuk_keadaan_ = this->now();
  }

  double lamaDiKeadaan() const
  {
    return (this->now() - masuk_keadaan_).seconds();
  }

  // ----------------------------------------------------------------
  // Pilih sisi untuk menghindar.
  //
  // Dua syarat harus dipenuhi sekaligus:
  //   1. Ruang kosong - LiDAR sektor sisi itu harus lega
  //   2. Garis pembatas boleh dilintasi - garis PUTUS-PUTUS memisahkan dua
  //      lajur sehingga boleh dilewati; garis SOLID adalah tepi jalan dan
  //      melewatinya berarti keluar lintasan
  //
  // Mengembalikan -1 untuk kiri, +1 untuk kanan, 0 bila tidak ada sisi
  // yang memenuhi syarat.
  // ----------------------------------------------------------------
  double jarakSisi(int arah) const
  {
    return (arah < 0) ? sek_kiri_ : sek_kanan_;
  }

  bool sisiLega(int arah) const
  {
    return jarakSisi(arah) > jarak_bebas_;
  }

  // Sebuah sisi boleh dipakai hanya bila ruangnya lega DAN pembatasnya
  // bukan garis solid.
  //
  // Garis solid adalah tepi jalan dan tidak boleh dilintasi dalam keadaan
  // apa pun. Hanya garis putus-putus, yang memisahkan dua lajur, yang boleh
  // diseberangi. Bila tidak ada sisi yang memenuhi syarat, mobil berhenti -
  // itu lebih baik daripada keluar lintasan.
  bool sisiBoleh(int arah) const
  {
    if (!sisiLega(arah)) return false;

    int jenis = (arah < 0) ? jenis_kiri_ : jenis_kanan_;
    if (hormati_garis_ && jenis == 2) return false;

    return true;
  }

  int pilihArah() const
  {
    // Menyingkir ke sisi yang BERLAWANAN dengan posisi halangan.
    // Memilih berdasarkan "sisi mana yang lega" adalah kesalahan: sisi yang
    // ditempati halangan justru akan tampak lega begitu halangannya sendiri
    // tidak lagi terhitung di sektor itu.
    int utama = (samping_depan_ >= 0.0) ? +1 : -1;   // halangan di kiri -> ke kanan

    if (sisiBoleh(utama))  return utama;
    if (sisiBoleh(-utama)) return -utama;
    return 0;
  }

  double besarGeseran(int) const { return offset_hindar_; }

  // ----------------------------------------------------------------
  // Batas keras: target kemudi tidak boleh berada di seberang garis solid.
  //
  // Melarang ARAH menghindar saja tidak cukup. Kalau pembacaan garis
  // sepotong membuat FSM salah menduga, atau geseran menumpuk dengan
  // lengkungan lajur, mobil tetap bisa terlempar melewati tepi jalan.
  // Pemotongan di sini berlaku apa pun keadaan FSM-nya, sehingga larangan
  // itu menjadi jaminan, bukan sekadar harapan.
  // ----------------------------------------------------------------
  // Apakah badan mobil sudah terlalu dekat dengan garis solid?
  //
  // Posisi mobil di gambar BEV adalah titik tengahnya. Jadi jarak mobil ke
  // garis solid bisa dibaca langsung sebagai selisih piksel. Pemotongan
  // target hanya mencegah mobil menuju ke seberang garis; ia tidak bisa
  // menarik mobil yang sudah terlanjur berada di sana.
  bool terlaluDekatGarisSolid() const
  {
    if (!hormati_garis_) return false;

    const double tengah = lebar_gambar_ / 2.0;

    if (jenis_kiri_ == 2 && x_kiri_ >= 0.0 &&
        (tengah - x_kiri_) < ambang_garis_) return true;

    if (jenis_kanan_ == 2 && x_kanan_ >= 0.0 &&
        (x_kanan_ - tengah) < ambang_garis_) return true;

    return false;
  }

  double batasiGarisSolid(double target) const
  {
    if (!hormati_garis_) return target;

    if (jenis_kiri_ == 2 && x_kiri_ >= 0.0) {
      target = std::max(target, x_kiri_ + margin_garis_);
    }
    if (jenis_kanan_ == 2 && x_kanan_ >= 0.0) {
      target = std::min(target, x_kanan_ - margin_garis_);
    }
    return target;
  }

  const char * namaGaris(int jenis) const
  {
    return jenis == 2 ? "solid" : jenis == 1 ? "putus" : "-";
  }

  // ----------------------------------------------------------------
  // Apakah mobil sedang berada di lajur seberang?
  //
  // Setelah menyeberang, vision ikut mengunci ke lajur yang baru. Jadi
  // mengembalikan geseran ke nol tidak membawa mobil pulang - ia hanya
  // duduk di tengah lajur yang salah. Yang membedakan keduanya adalah
  // SUSUNAN garisnya: di lajur yang benar garis putus-putus ada di satu
  // sisi tertentu, dan di lajur seberang susunan itu terbalik.
  //
  // Hanya dinilai bila kedua garis benar-benar terbaca.
  // ----------------------------------------------------------------
  bool diLajurSalah() const
  {
    // Satu garis saja sudah cukup menjadi bukti. Di lajur yang benar,
    // garis solid mustahil berada di sisi tempat seharusnya ada garis
    // putus-putus, dan sebaliknya. Menuntut kedua garis terbaca sekaligus
    // membuat penilaian ini hampir tidak pernah berjalan, karena garis
    // putus-putus memang sering hilang dari pandangan.
    if (putus_di_kiri_) {
      if (jenis_kiri_  == 2) return true;   // solid di kiri
      if (jenis_kanan_ == 1) return true;   // putus-putus di kanan
    } else {
      if (jenis_kanan_ == 2) return true;
      if (jenis_kiri_  == 1) return true;
    }
    return false;
  }

  // Arah pulang: menuju sisi tempat garis putus-putus terlihat sekarang,
  // karena garis itulah yang harus diseberangi kembali. Bila garis itu
  // sedang tidak terlihat, pulang berarti kebalikan arah menyingkir tadi.
  // Batas aman: dorongan pulang berhenti setelah jarak tertentu, supaya
  // pembacaan garis yang keliru tidak membuat mobil mendorong tanpa henti.
  bool pulangHabis() const
  {
    return (jarak_tempuh_ - jarak_saat_bersih_) > (jarak_lewat_ + batas_pulang_);
  }

  int arahPulang() const
  {
    if (jenis_kanan_ == 1) return +1;
    if (jenis_kiri_  == 1) return -1;
    return -arah_hindar_;
  }

  // --- FSM: tentukan keadaan berikutnya ---
  //
  // Dua mekanisme menjaga FSM tidak bergetar:
  //   1. Histeresis - ambang untuk MASUK dan KELUAR dari satu keadaan
  //      sengaja dibuat berbeda, tidak memakai angka yang sama.
  //   2. Dwell - setiap keadaan wajib bertahan minimal beberapa saat
  //      sebelum boleh berpindah, kecuali menuju BERHENTI.
  //
  void perbaruiKeadaan()
  {
    if (!hindar_aktif_ || !ada_sektor_) {
      setKeadaan(Keadaan::IKUTI_LAJUR);
      return;
    }

    const bool boleh_pindah = (lamaDiKeadaan() > dwell_min_);

    // Pelanggaran garis solid diperiksa lebih dulu daripada apa pun.
    // Keadaan ini boleh menyela keadaan mana pun, karena keluar lintasan
    // lebih buruk daripada gagal menghindar.
    if (keadaan_ != Keadaan::KOREKSI && terlaluDekatGarisSolid()) {
      setKeadaan(Keadaan::KOREKSI);
      jarak_mulai_koreksi_ = jarak_tempuh_;
      RCLCPP_WARN(this->get_logger(),
                  "Terlalu dekat garis solid (kiri %s, kanan %s). "
                  "Mundur dan menata ulang.",
                  namaGaris(jenis_kiri_), namaGaris(jenis_kanan_));
      return;
    }

    switch (keadaan_) {
      case Keadaan::IKUTI_LAJUR:
        if (sek_depan_ < jarak_hindar_) {
          int arah = pilihArah();
          if (arah != 0) {
            arah_hindar_ = arah;
            besar_geser_ = besarGeseran(arah);
            // Sisi tempat halangan berada, dipakai nanti untuk mengetahui
            // kapan mobil benar-benar sudah melewatinya
            sisi_halangan_ = (samping_depan_ >= 0.0) ? -1 : +1;
            setKeadaan(Keadaan::MENGHINDAR);
            RCLCPP_INFO(this->get_logger(),
                        "Halangan %.1f m di sisi %s (%.2f m). Menyingkir ke %s "
                        "sejauh %.0f px. Kiri: %.1f m garis %s. "
                        "Kanan: %.1f m garis %s.",
                        sek_depan_,
                        samping_depan_ >= 0.0 ? "kiri" : "kanan", samping_depan_,
                        arah < 0 ? "KIRI" : "KANAN", besar_geser_,
                        sek_kiri_, namaGaris(jenis_kiri_),
                        sek_kanan_, namaGaris(jenis_kanan_));
          } else if (sek_depan_ < obstacle_stop_) {
            setKeadaan(Keadaan::BERHENTI);
            RCLCPP_WARN(this->get_logger(),
                        "Halangan %.1f m, tidak ada sisi yang boleh dilewati "
                        "(kiri garis %s, kanan garis %s). Berhenti.",
                        sek_depan_, namaGaris(jenis_kiri_),
                        namaGaris(jenis_kanan_));
          }
        }
        break;

      case Keadaan::MENGHINDAR:
        // Jalur buntu tetap diperiksa tanpa menunggu dwell
        if (sek_depan_ < stop_saat_hindar_) {
          setKeadaan(Keadaan::BERHENTI);
          RCLCPP_WARN(this->get_logger(),
                      "Terlalu dekat (%.1f m) saat menghindar. Berhenti.",
                      sek_depan_);
          break;
        }
        // Ambang keluar lebih longgar daripada ambang masuk (histeresis)
        if (boleh_pindah && sek_depan_ > jarak_hindar_ * 1.6) {
          setKeadaan(Keadaan::KEMBALI);
          jarak_saat_bersih_   = jarak_tempuh_;
          jarak_masuk_kembali_ = jarak_tempuh_;
          RCLCPP_INFO(this->get_logger(),
                      "Jalur bersih (%.1f m). Menahan geseran sampai melewati halangan.",
                      sek_depan_);
        }
        break;

      case Keadaan::KEMBALI: {
        if (sek_depan_ < stop_saat_hindar_) {
          setKeadaan(Keadaan::BERHENTI);
          break;
        }

        // Selama halangan masih terbaca di samping mobil, titik "sudah
        // bersih" terus digeser maju. Jadi hitungan jarak pulang baru
        // mulai berjalan setelah halangan benar-benar terlewati.
        //
        // Tanpa ini, hitungannya dimulai saat jalur depan bersih - yaitu
        // ketika mobil masih beberapa meter SEBELUM halangan - sehingga
        // mobil membelok kembali tepat saat sejajar dengannya.
        bool masih_di_samping =
            (jarakSisi(sisi_halangan_) < jarak_bebas_) &&
            ((jarak_tempuh_ - jarak_masuk_kembali_) < batas_tahan_);

        if (masih_di_samping) jarak_saat_bersih_ = jarak_tempuh_;

        const bool masih_menahan =
            (jarak_tempuh_ - jarak_saat_bersih_) < jarak_lewat_;

        // Selama masih menahan geseran, halangan di depan diabaikan.
        // Inilah yang mencegah FSM membalik ke MENGHINDAR berulang kali
        // hanya karena halangan yang sedang dilewati masih terlihat.
        // Halangan berikutnya baru dilayani kalau sudah BENAR-BENAR dekat.
        // Ambangnya lebih ketat daripada saat menyusuri lajur biasa, supaya
        // mobil sempat pulang dulu ke lajur asalnya alih-alih bertahan di
        // seberang hanya karena ada sesuatu di kejauhan.
        int arah_baru = (!masih_menahan && boleh_pindah &&
                         sek_depan_ < jarak_hindar_lagi_)
                      ? pilihArah() : 0;

        if (arah_baru != 0) {
          arah_hindar_ = arah_baru;
          besar_geser_ = besarGeseran(arah_baru);
          setKeadaan(Keadaan::MENGHINDAR);
          RCLCPP_INFO(this->get_logger(), "Halangan lain muncul. Menghindar lagi.");
        } else if (!masih_menahan && std::fabs(geser_) < 5.0 &&
                   (!diLajurSalah() || pulangHabis())) {
          setKeadaan(Keadaan::IKUTI_LAJUR);
          RCLCPP_INFO(this->get_logger(),
                      "Halangan terlewati. Sudah kembali ke lajur "
                      "(kiri %s, kanan %s).",
                      namaGaris(jenis_kiri_), namaGaris(jenis_kanan_));
        }
        break;
      }

      case Keadaan::KOREKSI: {
        const double mundur = jarak_tempuh_ - jarak_mulai_koreksi_;

        // Keluar bila sudah menjauh dari garis, atau bila sudah mundur
        // cukup jauh tanpa perbaikan - daripada mundur tanpa henti.
        if ((!terlaluDekatGarisSolid() && mundur > 1.0) ||
            mundur > jarak_koreksi_) {
          geser_ = 0.0;
          setKeadaan(Keadaan::IKUTI_LAJUR);
          RCLCPP_INFO(this->get_logger(),
                      "Sudah menjauh dari garis solid. Lanjut menyusuri lajur.");
        }
        break;
      }

      case Keadaan::BERHENTI: {
        // Jalur benar-benar bersih
        if (sek_depan_ > obstacle_stop_ * 1.4) {
          setKeadaan(Keadaan::IKUTI_LAJUR);
          RCLCPP_INFO(this->get_logger(), "Jalur bersih. Jalan lagi.");
          break;
        }

        // Ruang sudah cukup untuk bermanuver dan ada sisi yang bisa dilewati
        int arah = pilihArah();
        if (arah != 0 && sek_depan_ > stop_saat_hindar_ * 1.5) {
          arah_hindar_ = arah;
          besar_geser_ = besarGeseran(arah);
          setKeadaan(Keadaan::MENGHINDAR);
          RCLCPP_INFO(this->get_logger(),
                      "Ruang cukup (%.1f m). Mencoba menyingkir ke %s.",
                      sek_depan_, arah < 0 ? "KIRI" : "KANAN");
        }
        break;
      }
    }
  }

  // --- Geseran target, dinaikkan/diturunkan bertahap ---
  void perbaruiGeseran(double dt)
  {
    double tujuan = 0.0;

    if (keadaan_ == Keadaan::MENGHINDAR) {
      tujuan = arah_hindar_ * besar_geser_;
    } else if (keadaan_ == Keadaan::KEMBALI) {
      if (jarak_tempuh_ - jarak_saat_bersih_ < jarak_lewat_) {
        // Tahan geseran sampai buritan benar-benar melewati halangan
        tujuan = arah_hindar_ * besar_geser_;
      } else if (diLajurSalah() && !pulangHabis()) {
        // Masih di lajur seberang. Melepas geseran saja tidak membawa
        // mobil pulang, karena vision sudah mengunci ke lajur yang baru -
        // mobil hanya akan duduk rapi di tengah lajur yang salah.
        //
        // Dorongan ini dulu sempat melempar mobil melewati garis solid.
        // Sekarang aman karena target selalu dipotong oleh batasiGarisSolid
        // dan keadaan KOREKSI menangkap pelanggaran yang lolos.
        tujuan = arahPulang() * besar_geser_;
      } else {
        tujuan = 0.0;
      }
    }

    double maks = laju_ramp_ * dt;
    double beda = tujuan - geser_;
    geser_ += std::clamp(beda, -maks, maks);
  }

  // --- Objektif 6 + penghindaran ---
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

    perbaruiKeadaan();
    perbaruiGeseran(0.05);   // dipanggil tiap 50 ms

    if (keadaan_ == Keadaan::KOREKSI) {
      // Mundur lurus. Menyetir saat mundur membuat buritan melebar ke arah
      // yang sulit diperkirakan, jadi lebih aman menarik mobil kembali
      // menyusuri jalurnya sendiri lalu membiarkan pengikut lajur menata
      // arahnya setelah maju lagi.
      kirim(pwm_mundur_, 0);
      return;
    }

    if (keadaan_ == Keadaan::BERHENTI) {
      // Terlalu mepet untuk bermanuver: mundur pelan agar dapat ruang.
      // Tanpa ini mobil terkunci selamanya, karena syarat keluar dari
      // BERHENTI adalah halangan menjauh, padahal mobilnya diam.
      bool ada_jalan = (pilihArah() != 0);
      if (ada_jalan && lamaDiKeadaan() > sabar_berhenti_ &&
          sek_depan_ < stop_saat_hindar_ * 1.5) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Terlalu mepet (%.1f m). Mundur untuk cari ruang.",
                             sek_depan_);
        kirim(pwm_mundur_, 0);
      } else {
        kirim(0, 0);
      }
      return;
    }

    double tengah = lebar_gambar_ / 2.0;
    double target = batasiGarisSolid(turn_target_x_ + geser_);

    offset_ = target - tengah;
    double norm = std::clamp(offset_ / tengah, -1.0, 1.0);

    double steering = norm * follow_gain_;

    // Saat menyingkir, tambahkan dorongan setir langsung.
    //
    // Menggeser target saja tidak cukup: pengikut lajur menarik mobil kembali
    // ke tengah lajur, dan kedua suku itu bisa saling meniadakan sampai setir
    // nyaris lurus. Dorongan ini menjamin mobil benar-benar bergerak ke
    // samping, bukan sekadar "berniat" bergeser.
    if (keadaan_ == Keadaan::MENGHINDAR) {
      steering += arah_hindar_ * bias_hindar_;
    }

    steering = std::clamp(steering, -90.0, 90.0);

    int pwm = static_cast<int>(pwm_jalan_ * (1.0 - 0.6 * std::fabs(norm)));

    // Lebih pelan saat sedang bermanuver, agar geseran sempat terjadi
    // sebelum mobil sampai ke halangan
    if (keadaan_ == Keadaan::MENGHINDAR || keadaan_ == Keadaan::KEMBALI) {
      pwm = static_cast<int>(pwm * 0.6);
    }

    pwm = std::clamp(std::max(pwm, pwm_min_), -255, 255);

    kirim(pwm, static_cast<int>(steering));
  }

  // --- Objektif 3 ---
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
  // Satu-satunya jalan keluar perintah, sekaligus lapisan pengaman.
  //
  // Pengaman ini bekerja terpisah dari FSM. Kalau FSM salah menilai
  // keadaan, lapisan ini tetap mengerem. Ambangnya dilonggarkan saat
  // sedang menghindar, supaya mobil tidak berhenti justru ketika sedang
  // dalam proses menyingkir.
  // ----------------------------------------------------------------
  void kirim(int pwm, int steering)
  {
    rem_aktif_ = false;

    if (ada_obstacle_ && pwm > 0) {
      double umur_ms = (this->now() - last_obstacle_time_).nanoseconds() / 1.0e6;

      if (umur_ms < obstacle_timeout_ms_) {
        // MENGHINDAR dan KEMBALI sama-sama keadaan bermanuver. Keduanya
        // harus memakai ambang longgar: di KEMBALI mobil justru sedang
        // menahan geseran untuk melewati halangan, jadi mengeremnya di situ
        // membuat manuver tidak pernah selesai dan mobil terkunci.
        const bool bermanuver = (keadaan_ == Keadaan::MENGHINDAR ||
                                 keadaan_ == Keadaan::KEMBALI);

        double ambang_stop = bermanuver ? stop_saat_hindar_ : obstacle_stop_;

        if (obstacle_dist_ < ambang_stop) {
          pwm = 0;
          rem_aktif_ = true;
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "HALANGAN %.2f m - berhenti.", obstacle_dist_);
        } else if (!bermanuver && obstacle_dist_ < obstacle_slow_) {
          double f = (obstacle_dist_ - ambang_stop) /
                     std::max(0.1, obstacle_slow_ - ambang_stop);
          pwm = static_cast<int>(pwm * std::clamp(f, 0.0, 1.0));
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

    char info[160];
    if (!ada_sektor_) {
      std::snprintf(info, sizeof(info), " | lidar: -");
    } else {
      std::snprintf(info, sizeof(info),
                    " | lidar D%.1f(%+.1f) K%.1f N%.1f | garis K:%s N:%s%s%s",
                    sek_depan_, samping_depan_, sek_kiri_, sek_kanan_,
                    namaGaris(jenis_kiri_), namaGaris(jenis_kanan_),
                    diLajurSalah() ? " [LAJUR SEBERANG]" : "",
                    rem_aktif_ ? " [REM]" : "");
    }

    if (mode_ == "follow") {
      RCLCPP_INFO(this->get_logger(),
                  "%-8s | offset=%+.0f px geser=%+.0f %s | pwm=%d steer=%d%s",
                  namaKeadaan(keadaan_), offset_, geser_,
                  turn_valid_ ? "[lihat]" : "[tebak]",
                  last_pwm_, last_steering_, info);
    } else {
      double tx = target_relatif_ ? world_target_x_ : target_x_;
      double ty = target_relatif_ ? world_target_y_ : target_y_;
      RCLCPP_INFO(this->get_logger(),
                  "POINT | pos=(%.2f, %.2f) hdg=%.0f | target=(%.1f, %.1f) "
                  "jarak=%.2f m | pwm=%d steer=%d%s%s",
                  x_, y_, theta_ * 180.0 / M_PI, tx, ty,
                  jarak_, last_pwm_, last_steering_,
                  sampai_ ? "  [SAMPAI]" : "", info);
    }
  }

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_feedback_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_target_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_obstacle_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_sektor_;
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

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_lane_;
  bool   ada_sektor_ = false;
  double sek_depan_ = 99.0, sek_kiri_ = 99.0, sek_kanan_ = 99.0;

  bool   ada_lane_ = false;
  int    jenis_kiri_ = 0, jenis_kanan_ = 0;   // 0 tidak ada, 1 putus, 2 solid
  double x_kiri_ = -1.0, x_kanan_ = -1.0;     // posisi garis, piksel BEV
  double margin_garis_, ambang_garis_, jarak_koreksi_;
  double jarak_mulai_koreksi_ = 0.0;
  bool   hormati_garis_;
  bool   putus_di_kiri_;
  double batas_pulang_;

  bool   hindar_aktif_;
  double jarak_hindar_, jarak_hindar_lagi_;
  double jarak_bebas_, offset_hindar_, laju_ramp_, jarak_lewat_;
  double besar_geser_ = 0.0, bias_hindar_;
  double samping_depan_ = 0.0;      // positif = halangan ada di sisi kiri
  double stop_saat_hindar_, dwell_min_, sabar_berhenti_;
  int    pwm_mundur_;
  Keadaan keadaan_ = Keadaan::IKUTI_LAJUR;
  rclcpp::Time masuk_keadaan_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  int    arah_hindar_ = 1;          // -1 = kiri, +1 = kanan
  double geser_ = 0.0;              // geseran target saat ini, piksel
  double jarak_tempuh_ = 0.0;
  double jarak_saat_bersih_ = 0.0;
  double jarak_masuk_kembali_ = 0.0;
  double batas_tahan_;
  int    sisi_halangan_ = -1;   // -1 = halangan di kiri, +1 = di kanan

  int pwm_jalan_, pwm_min_;

  double x_, y_, theta_;
  int32_t prev_left_, prev_right_;
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