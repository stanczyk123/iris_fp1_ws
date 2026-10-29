// IRIS FP1 - Vision Node
// Step 4: Image Thresholding & Bird-Eye View (BEV)
// Step 5: Finding Turn Target  (deteksi dua garis lajur)
//
// Alur: kamera -> threshold HSV -> warp BEV -> cari garis kiri & kanan
//       -> titik tengah lajur -> /turn_target
//
// PERUBAHAN DARI VERSI SEBELUMNYA
// Versi lama mengambil titik berat SELURUH piksel putih di pita pencarian.
// Cara itu gagal di tikungan: garis solid tepi jalan punya jauh lebih banyak
// piksel daripada garis putus-putus pembatas tengah, sehingga titik beratnya
// tertarik ke garis solid dan mobil keluar dari lajurnya.
//
// Versi ini mencari garis KIRI dan KANAN secara terpisah, lalu mengambil
// titik tengah di antara keduanya. Kalau hanya satu garis yang terlihat,
// posisi lajur diperkirakan dari garis itu ditambah setengah lebar lajur.
//
// Tiap garis juga diklasifikasikan PUTUS-PUTUS atau SOLID, dihitung dari
// rasio keterisian kolomnya sepanjang gambar BEV. Garis solid terisi hampir
// penuh; garis putus-putus tidak. Informasi ini dipakai saat hanya satu
// garis terlihat, untuk menentukan ke arah mana lajur berada.
//
// Hasil dipublish ke /turn_target (geometry_msgs/Point):
//   x = posisi target (piksel, 0..lebar BEV)
//   y = baris lookahead (piksel)
//   z = 1.0 kalau target valid, 0.0 kalau memakai nilai terakhir
//
// Tekan 's' untuk print parameter, 'q' untuk keluar.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>

namespace
{
const std::string CAMERA_TOPIC = "/catvehicle/camera_front/image_raw_front";
const std::string TARGET_TOPIC = "/turn_target";

const std::string WIN_TUNING = "Tuning";
const std::string WIN_RAW    = "1 - Raw Camera";
const std::string WIN_THRESH = "2 - Threshold";
const std::string WIN_BEV    = "3 - Bird Eye View";
}  // namespace

struct TuningParams
{
  int h_min, h_max;
  int s_min, s_max;
  int v_min, v_max;
  int top_w, bot_w;
  int top_y, bot_y;
  int look_y;        // posisi pita pencarian, % dari tinggi BEV
  int band_h;        // tebal pita, % dari tinggi BEV
  int lebar_lajur;   // setengah lebar lajur, % dari lebar BEV
  int min_piksel;    // piksel minimum agar sebuah garis dianggap ada
};

// Hasil deteksi satu sisi garis
struct Garis
{
  bool   ada   = false;
  double x     = 0.0;
  double isi   = 0.0;   // rasio keterisian kolom, 0..1
  bool   solid = false;
};

class VisionNode : public rclcpp::Node
{
public:
  VisionNode() : Node("vision_node"), frame_count_(0),
                 last_target_x_(-1.0), has_last_target_(false)
  {
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      CAMERA_TOPIC, rclcpp::SensorDataQoS(),
      std::bind(&VisionNode::imageCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::Point>(TARGET_TOPIC, 10);

    buildTuningWindow();

    RCLCPP_INFO(this->get_logger(), "Vision node aktif.");
    RCLCPP_INFO(this->get_logger(), "  subscribe : %s", CAMERA_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "  publish   : %s", TARGET_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "Menunggu gambar pertama...");
  }

  ~VisionNode() { cv::destroyAllWindows(); }

private:
  void buildTuningWindow()
  {
    cv::namedWindow(WIN_TUNING, cv::WINDOW_NORMAL);
    cv::resizeWindow(WIN_TUNING, 460, 460);

    // Nilai hasil kalibrasi
    p_.h_min = 0;   p_.h_max = 179;
    p_.s_min = 0;   p_.s_max = 40;
    p_.v_min = 180; p_.v_max = 255;

    p_.top_w = 20;  p_.bot_w = 95;
    p_.top_y = 60;  p_.bot_y = 95;

    p_.look_y = 50;
    p_.band_h = 15;

    // Setengah lebar lajur dalam BEV. Perlu dituning sekali: jalankan di
    // jalan lurus saat kedua garis terlihat, lihat nilai "setengah lebar"
    // yang tercetak, lalu set slider ini ke angka itu.
    p_.lebar_lajur = 25;
    p_.min_piksel  = 60;

    cv::createTrackbar("H min", WIN_TUNING, &p_.h_min, 179);
    cv::createTrackbar("H max", WIN_TUNING, &p_.h_max, 179);
    cv::createTrackbar("S min", WIN_TUNING, &p_.s_min, 255);
    cv::createTrackbar("S max", WIN_TUNING, &p_.s_max, 255);
    cv::createTrackbar("V min", WIN_TUNING, &p_.v_min, 255);
    cv::createTrackbar("V max", WIN_TUNING, &p_.v_max, 255);

    cv::createTrackbar("BEV atas lebar %",  WIN_TUNING, &p_.top_w, 100);
    cv::createTrackbar("BEV bawah lebar %", WIN_TUNING, &p_.bot_w, 100);
    cv::createTrackbar("BEV atas Y %",      WIN_TUNING, &p_.top_y, 100);
    cv::createTrackbar("BEV bawah Y %",     WIN_TUNING, &p_.bot_y, 100);

    cv::createTrackbar("Lookahead Y %",     WIN_TUNING, &p_.look_y, 100);
    cv::createTrackbar("Tebal pita %",      WIN_TUNING, &p_.band_h, 50);
    cv::createTrackbar("1/2 lebar lajur %", WIN_TUNING, &p_.lebar_lajur, 60);
    cv::createTrackbar("Min piksel",        WIN_TUNING, &p_.min_piksel, 400);
  }

  cv::Mat threshold(const cv::Mat & frame)
  {
    cv::Mat blur, hsv, mask;

    cv::GaussianBlur(frame, blur, cv::Size(5, 5), 0);
    cv::cvtColor(blur, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv,
                cv::Scalar(p_.h_min, p_.s_min, p_.v_min),
                cv::Scalar(p_.h_max, p_.s_max, p_.v_max),
                mask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel, cv::Point(-1, -1), 1);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2);

    return mask;
  }

  std::vector<cv::Point2f> getBevPoints(int w, int h)
  {
    float top_w = p_.top_w / 100.0f;
    float bot_w = p_.bot_w / 100.0f;
    float top_y = p_.top_y / 100.0f * h;
    float bot_y = p_.bot_y / 100.0f * h;

    float cx = w / 2.0f;

    return {
      cv::Point2f(cx - (top_w * w / 2.0f), top_y),
      cv::Point2f(cx + (top_w * w / 2.0f), top_y),
      cv::Point2f(cx + (bot_w * w / 2.0f), bot_y),
      cv::Point2f(cx - (bot_w * w / 2.0f), bot_y)
    };
  }

  cv::Mat warpBev(const cv::Mat & img, const std::vector<cv::Point2f> & src,
                  int w, int h)
  {
    std::vector<cv::Point2f> dst = {
      cv::Point2f(0, 0),
      cv::Point2f(static_cast<float>(w), 0),
      cv::Point2f(static_cast<float>(w), static_cast<float>(h)),
      cv::Point2f(0, static_cast<float>(h))
    };

    cv::Mat matrix = cv::getPerspectiveTransform(src, dst);
    cv::Mat bev;
    cv::warpPerspective(img, bev, matrix, cv::Size(w, h));
    return bev;
  }

  // ----------------------------------------------------------------
  // Cari satu garis di dalam rentang kolom [x_awal, x_akhir)
  //
  // Dua tahap supaya tahan terhadap noise sekaligus halus:
  //   1. cari kolom dengan piksel putih terbanyak (puncak histogram)
  //   2. hitung titik berat di sekitar puncak itu saja
  //
  // Memakai titik berat seluruh rentang akan tertarik oleh noise di tepi;
  // memakai puncak saja menghasilkan posisi yang meloncat-loncat.
  // ----------------------------------------------------------------
  Garis cariGaris(const cv::Mat & bev, const cv::Mat & band,
                  int x_awal, int x_akhir)
  {
    Garis g;
    const int w = bev.cols;
    const int h = bev.rows;

    x_awal  = std::clamp(x_awal, 0, w);
    x_akhir = std::clamp(x_akhir, 0, w);
    if (x_akhir - x_awal < 3) return g;

    // --- Tahap 1: histogram kolom ---
    std::vector<int> hist(x_akhir - x_awal, 0);
    for (int y = 0; y < band.rows; ++y) {
      const uint8_t * baris = band.ptr<uint8_t>(y);
      for (int x = x_awal; x < x_akhir; ++x) {
        if (baris[x] > 0) hist[x - x_awal]++;
      }
    }

    int puncak_i = 0, puncak_v = 0, total = 0;
    for (size_t i = 0; i < hist.size(); ++i) {
      total += hist[i];
      if (hist[i] > puncak_v) { puncak_v = hist[i]; puncak_i = static_cast<int>(i); }
    }

    if (total < p_.min_piksel || puncak_v == 0) return g;

    // --- Tahap 2: titik berat di sekitar puncak ---
    const int jendela = std::max(5, w / 20);
    int lo = std::max(0, puncak_i - jendela);
    int hi = std::min(static_cast<int>(hist.size()), puncak_i + jendela + 1);

    double jumlah = 0.0, berbobot = 0.0;
    for (int i = lo; i < hi; ++i) {
      jumlah   += hist[i];
      berbobot += hist[i] * static_cast<double>(i + x_awal);
    }
    if (jumlah < 1.0) return g;

    g.ada = true;
    g.x   = berbobot / jumlah;

    // --- Klasifikasi putus-putus vs solid ---
    // Hitung keterisian kolom di sekitar garis, sepanjang SELURUH tinggi BEV.
    // Garis solid terisi hampir penuh, garis putus-putus hanya sebagian.
    int cx  = static_cast<int>(g.x);
    int clo = std::clamp(cx - jendela, 0, w);
    int chi = std::clamp(cx + jendela, 0, w);

    int terisi = 0;
    for (int y = 0; y < h; ++y) {
      const uint8_t * baris = bev.ptr<uint8_t>(y);
      for (int x = clo; x < chi; ++x) {
        if (baris[x] > 0) { terisi++; break; }   // cukup satu per baris
      }
    }
    g.isi   = static_cast<double>(terisi) / std::max(1, h);
    g.solid = (g.isi > 0.75);

    return g;
  }

  // ----------------------------------------------------------------
  // Step 5: tentukan titik tengah lajur
  // ----------------------------------------------------------------
  bool cariTargetLajur(const cv::Mat & bev, int & band_top, int & band_bot,
                       double & target_x, Garis & kiri, Garis & kanan)
  {
    const int w = bev.cols;
    const int h = bev.rows;

    int band_center = static_cast<int>(p_.look_y / 100.0 * h);
    int half_band   = std::max(1, static_cast<int>(p_.band_h / 100.0 * h / 2));

    band_top = std::max(0, band_center - half_band);
    band_bot = std::min(h, band_center + half_band);
    if (band_bot <= band_top) return false;

    cv::Mat band = bev(cv::Range(band_top, band_bot), cv::Range::all());

    const int tengah = w / 2;
    kiri  = cariGaris(bev, band, 0, tengah);
    kanan = cariGaris(bev, band, tengah, w);

    double setengah_lajur = p_.lebar_lajur / 100.0 * w;

    if (kiri.ada && kanan.ada) {
      // Kasus ideal: kedua garis terlihat, lajur ada persis di tengahnya
      target_x = (kiri.x + kanan.x) / 2.0;
      lebar_terukur_ = (kanan.x - kiri.x) / 2.0;
      return true;
    }

    if (kiri.ada) {
      // Hanya garis kiri. Lajur berada setengah lebar ke kanan dari situ.
      target_x = kiri.x + setengah_lajur;
      return true;
    }

    if (kanan.ada) {
      target_x = kanan.x - setengah_lajur;
      return true;
    }

    return false;   // tidak ada garis sama sekali
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Gagal convert gambar: %s", e.what());
      return;
    }

    if (frame.empty()) return;

    if (frame_count_ == 0) {
      RCLCPP_INFO(this->get_logger(), "Gambar pertama diterima! Ukuran: %dx%d",
                  frame.cols, frame.rows);
    }
    frame_count_++;

    const int w = frame.cols;
    const int h = frame.rows;

    cv::Mat mask = threshold(frame);

    std::vector<cv::Point2f> src = getBevPoints(w, h);
    cv::Mat bev = warpBev(mask, src, w, h);

    int band_top = 0, band_bot = 0;
    double target_x = 0.0;
    Garis kiri, kanan;
    bool found = cariTargetLajur(bev, band_top, band_bot, target_x, kiri, kanan);

    bool valid = found;
    if (found) {
      last_target_x_   = target_x;
      has_last_target_ = true;
    } else if (has_last_target_) {
      target_x = last_target_x_;
    } else {
      target_x = w / 2.0;
    }

    int band_center_y = (band_top + band_bot) / 2;

    auto point = geometry_msgs::msg::Point();
    point.x = target_x;
    point.y = static_cast<double>(band_center_y);
    point.z = valid ? 1.0 : 0.0;
    pub_->publish(point);

    // ---------------- Tampilan ----------------
    cv::Mat bev_color;
    cv::cvtColor(bev, bev_color, cv::COLOR_GRAY2BGR);

    cv::rectangle(bev_color, cv::Point(0, band_top), cv::Point(w, band_bot),
                  cv::Scalar(0, 255, 255), 2);

    // Garis tengah gambar = arah lurus mobil
    cv::line(bev_color, cv::Point(w / 2, 0), cv::Point(w / 2, h),
             cv::Scalar(255, 0, 0), 1);

    // Garis kiri (biru) dan kanan (merah), dengan label jenisnya
    if (kiri.ada) {
      int x = static_cast<int>(kiri.x);
      cv::line(bev_color, cv::Point(x, band_top), cv::Point(x, band_bot),
               cv::Scalar(255, 128, 0), 3);
      cv::putText(bev_color, kiri.solid ? "SOLID" : "PUTUS",
                  cv::Point(std::max(2, x - 35), band_top - 8),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 128, 0), 1);
    }
    if (kanan.ada) {
      int x = static_cast<int>(kanan.x);
      cv::line(bev_color, cv::Point(x, band_top), cv::Point(x, band_bot),
               cv::Scalar(0, 128, 255), 3);
      cv::putText(bev_color, kanan.solid ? "SOLID" : "PUTUS",
                  cv::Point(std::min(w - 70, x - 35), band_top - 8),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 128, 255), 1);
    }

    cv::Scalar warna = valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::circle(bev_color, cv::Point(static_cast<int>(target_x), band_center_y),
               10, warna, -1);
    cv::line(bev_color, cv::Point(w / 2, h),
             cv::Point(static_cast<int>(target_x), band_center_y), warna, 2);

    double offset = target_x - w / 2.0;
    cv::putText(bev_color, cv::format("offset: %+.1f px", offset),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 255), 2);

    const char * sumber = (kiri.ada && kanan.ada) ? "2 garis"
                        : kiri.ada                ? "kiri saja"
                        : kanan.ada               ? "kanan saja"
                                                  : "tidak ada";
    cv::putText(bev_color, cv::format("sumber: %s", sumber),
                cv::Point(10, 58), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 2);

    if (lebar_terukur_ > 0.0) {
      cv::putText(bev_color,
                  cv::format("1/2 lebar terukur: %.0f px (%.0f%%)",
                             lebar_terukur_, lebar_terukur_ / w * 100.0),
                  cv::Point(10, 84), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(200, 200, 200), 1);
    }

    cv::Mat preview = frame.clone();
    std::vector<cv::Point> poly;
    for (const auto & pt : src) {
      poly.emplace_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
    }
    cv::polylines(preview, poly, true, cv::Scalar(0, 0, 255), 2);

    cv::imshow(WIN_RAW, preview);
    cv::imshow(WIN_THRESH, mask);
    cv::imshow(WIN_BEV, bev_color);

    int key = cv::waitKey(1) & 0xFF;
    if (key == 's') {
      RCLCPP_INFO(this->get_logger(),
                  "HSV [%d,%d,%d]-[%d,%d,%d] | BEV %d/%d/%d/%d | "
                  "look_y=%d band_h=%d lebar_lajur=%d min_piksel=%d | "
                  "1/2 lebar terukur=%.0f px",
                  p_.h_min, p_.s_min, p_.v_min,
                  p_.h_max, p_.s_max, p_.v_max,
                  p_.top_w, p_.bot_w, p_.top_y, p_.bot_y,
                  p_.look_y, p_.band_h, p_.lebar_lajur, p_.min_piksel,
                  lebar_terukur_);
    } else if (key == 'q') {
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;

  TuningParams p_;
  unsigned long frame_count_;

  double last_target_x_;
  bool   has_last_target_;
  double lebar_terukur_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionNode>());
  rclcpp::shutdown();
  return 0;
}