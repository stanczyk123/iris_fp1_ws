// IRIS FP1 - Vision Node
// Step 4: Image Thresholding & Bird-Eye View (BEV)
// Step 5: Finding Turn Target  (deteksi dua garis lajur)
//
// Alur: kamera -> threshold HSV -> warp BEV -> cari garis lajur
//       -> titik tengah lajur -> /turn_target
//
// CATATAN PERUBAHAN
//
// v1 mengambil titik berat SELURUH piksel putih di pita pencarian. Gagal di
// tikungan karena garis solid punya jauh lebih banyak piksel daripada garis
// putus-putus, sehingga titik beratnya tertarik ke garis solid.
//
// v2 membagi gambar menjadi setengah kiri dan setengah kanan. Masih gagal di
// tikungan tajam, karena di sana KEDUA garis bisa berada di sisi yang sama
// dari tengah gambar, sehingga salah satu sisi tidak menemukan apa pun.
//
// v3 (versi ini) mencari SEMUA garis di seluruh lebar gambar, lalu memilih
// pasangan yang jaraknya paling mendekati lebar lajur. Bila hanya satu garis
// yang terlihat, sisi kiri/kanannya ditentukan dari kesinambungan dengan
// target sebelumnya, bukan dari posisi tengah gambar.
//
// Klasifikasi PUTUS-PUTUS vs SOLID dihitung dengan jendela geser yang
// MENGIKUTI lengkungan garis. Versi sebelumnya memakai kolom tegak lurus,
// sehingga garis solid yang miring salah terbaca sebagai putus-putus.
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
  int zoom_out;      // margin hasil warp, % dari lebar BEV
};

struct Garis
{
  double x    = 0.0;
  double isi  = 0.0;    // rasio keterisian, 0..1
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

    p_.h_min = 0;   p_.h_max = 179;
    p_.s_min = 0;   p_.s_max = 40;
    p_.v_min = 180; p_.v_max = 255;

    p_.top_w = 60;  p_.bot_w = 100;
    // Sisi bawah dinaikkan agar wadah sensor di kap mobil, yang berwarna
    // putih dan selalu ada di frame, tidak ikut ter-warp ke BEV.
    p_.top_y = 70;  p_.bot_y = 83;

    // Trapesium dipetakan ke daerah yang lebih sempit dari gambar keluaran,
    // sehingga area di LUAR trapesium ikut terlihat. Tanpa ini, lajur nyaris
    // selebar frame dan salah satu garis selalu mepet tepi — terpotong,
    // lalu salah diklasifikasi sebagai putus-putus.
    p_.zoom_out = 20;

    // Lookahead lebih dekat ke mobil (nilai lebih besar = lebih dekat).
    // Di tikungan tajam, titik yang terlalu jauh ke depan sudah keluar
    // dari lajur sebelum mobil sampai ke sana.
    p_.look_y = 62;
    // Pita harus cukup tinggi untuk memotong setidaknya satu segmen garis
    // putus-putus. Pita tipis sering jatuh tepat di jeda antar segmen.
    p_.band_h = 30;

    p_.lebar_lajur = 24;
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
    cv::createTrackbar("Zoom out %",        WIN_TUNING, &p_.zoom_out, 40);
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
    // Margin membuat trapesium dipetakan ke daerah yang lebih sempit.
    // warpPerspective tetap mengisi seluruh gambar keluaran memakai
    // homografi yang sama, jadi daerah di luar margin menampilkan area di
    // LUAR trapesium. Efeknya seperti memundurkan kamera: lajur jadi lebih
    // ramping di frame dan kedua garis punya ruang dari tepi.
    float m = p_.zoom_out / 100.0f * w;

    std::vector<cv::Point2f> dst = {
      cv::Point2f(m, 0),
      cv::Point2f(w - m, 0),
      cv::Point2f(w - m, static_cast<float>(h)),
      cv::Point2f(m, static_cast<float>(h))
    };

    cv::Mat matrix = cv::getPerspectiveTransform(src, dst);
    cv::Mat bev;
    cv::warpPerspective(img, bev, matrix, cv::Size(w, h));
    return bev;
  }

  // ----------------------------------------------------------------
  // Keterisian garis, diukur dengan jendela geser yang MENGIKUTI garis.
  //
  // Mulai dari posisi garis di pita, lalu merambat ke atas dan ke bawah.
  // Tiap langkah, jendela digeser ke titik berat piksel putih yang ditemukan,
  // sehingga lengkungan garis tetap terikuti. Rasio = langkah yang menemukan
  // piksel dibagi total langkah.
  //
  //   garis solid       -> mendekati 1.0
  //   garis putus-putus -> sekitar 0.4 - 0.7
  //
  // Kolom tegak lurus (versi sebelumnya) salah membaca garis solid yang
  // miring sebagai putus-putus, karena garisnya keluar dari kolom.
  // ----------------------------------------------------------------
  double keterisian(const cv::Mat & bev, double x_awal, int y_awal)
  {
    const int w = bev.cols;
    const int h = bev.rows;
    const int langkah = std::max(4, h / 40);
    const int jendela = std::max(6, w / 25);

    int total = 0, kena = 0;

    for (int arah = -1; arah <= 1; arah += 2) {
      double x = x_awal;
      for (int y = y_awal; y >= 0 && y < h; y += arah * langkah) {
        // Jangan menghitung langkah yang jendelanya menyentuh tepi gambar.
        // Garis yang terpotong frame akan terlihat "bolong" dan salah
        // diklasifikasi sebagai putus-putus padahal dia menerus.
        if (x < jendela || x > w - jendela) continue;

        total++;

        int xlo = std::clamp(static_cast<int>(x) - jendela, 0, w);
        int xhi = std::clamp(static_cast<int>(x) + jendela, 0, w);
        int ylo = std::max(0, y - langkah / 2);
        int yhi = std::min(h, y + langkah / 2 + 1);

        int cacah = 0;
        double jumlah = 0.0;
        for (int yy = ylo; yy < yhi; ++yy) {
          const uint8_t * baris = bev.ptr<uint8_t>(yy);
          for (int xx = xlo; xx < xhi; ++xx) {
            if (baris[xx] > 0) { cacah++; jumlah += xx; }
          }
        }

        if (cacah > 0) {
          kena++;
          x = jumlah / cacah;   // ikuti lengkungan
        }
      }
    }

    return total > 0 ? static_cast<double>(kena) / total : 0.0;
  }

  // ----------------------------------------------------------------
  // Cari SEMUA garis di seluruh lebar pita, bukan per setengah gambar.
  //
  // Di tikungan tajam kedua garis bisa berada di sisi yang sama dari
  // tengah gambar, sehingga pembagian kiri/kanan gagal total.
  // ----------------------------------------------------------------
  std::vector<Garis> cariSemuaGaris(const cv::Mat & bev, const cv::Mat & band,
                                    int band_center_y)
  {
    const int w = bev.cols;
    std::vector<Garis> hasil;

    // Histogram kolom
    std::vector<int> hist(w, 0);
    for (int y = 0; y < band.rows; ++y) {
      const uint8_t * baris = band.ptr<uint8_t>(y);
      for (int x = 0; x < w; ++x) {
        if (baris[x] > 0) hist[x]++;
      }
    }

    // Haluskan agar satu garis tidak terpecah jadi beberapa puncak
    const int k = std::max(3, w / 60);
    std::vector<int> halus(w, 0);
    for (int x = 0; x < w; ++x) {
      int jumlah = 0, n = 0;
      for (int i = -k; i <= k; ++i) {
        int xx = x + i;
        if (xx >= 0 && xx < w) { jumlah += hist[xx]; n++; }
      }
      halus[x] = n > 0 ? jumlah / n : 0;
    }

    // Ambil puncak tertinggi berulang kali, tiap kali menutup daerah di
    // sekitarnya supaya satu garis tidak terhitung dua kali.
    const int jarak_min = std::max(10, w / 12);
    std::vector<bool> dipakai(w, false);

    while (hasil.size() < 4) {
      int puncak = -1, nilai = 0;
      for (int x = 0; x < w; ++x) {
        if (!dipakai[x] && halus[x] > nilai) { nilai = halus[x]; puncak = x; }
      }
      if (puncak < 0 || nilai == 0) break;

      int lo = std::max(0, puncak - jarak_min);
      int hi = std::min(w, puncak + jarak_min + 1);

      int total = 0;
      double berbobot = 0.0;
      for (int x = lo; x < hi; ++x) {
        total    += hist[x];
        berbobot += hist[x] * static_cast<double>(x);
      }

      for (int x = lo; x < hi; ++x) dipakai[x] = true;

      if (total < p_.min_piksel) break;   // puncak berikutnya pasti lebih kecil

      Garis g;
      g.x     = berbobot / total;
      g.isi   = keterisian(bev, g.x, band_center_y);
      g.solid = (g.isi > 0.80);
      hasil.push_back(g);
    }

    std::sort(hasil.begin(), hasil.end(),
              [](const Garis & a, const Garis & b) { return a.x < b.x; });
    return hasil;
  }

  // ----------------------------------------------------------------
  // Tentukan titik tengah lajur dari garis-garis yang ditemukan
  // ----------------------------------------------------------------
  bool cariTargetLajur(const cv::Mat & bev, int & band_top, int & band_bot,
                       double & target_x, std::vector<Garis> & garis,
                       int & idx_kiri, int & idx_kanan)
  {
    const int w = bev.cols;
    const int h = bev.rows;

    int band_center = static_cast<int>(p_.look_y / 100.0 * h);
    int half_band   = std::max(1, static_cast<int>(p_.band_h / 100.0 * h / 2));

    band_top = std::max(0, band_center - half_band);
    band_bot = std::min(h, band_center + half_band);
    if (band_bot <= band_top) return false;

    cv::Mat band = bev(cv::Range(band_top, band_bot), cv::Range::all());

    garis = cariSemuaGaris(bev, band, (band_top + band_bot) / 2);
    idx_kiri = idx_kanan = -1;

    if (garis.empty()) return false;

    const double setengah = p_.lebar_lajur / 100.0 * w;
    const double harap    = 2.0 * setengah;

    // --- Dua garis atau lebih: pilih pasangan yang jaraknya paling
    //     mendekati lebar lajur ---
    if (garis.size() >= 2) {
      double beda_terbaik = 1e9;
      int bi = -1, bj = -1;

      for (size_t i = 0; i + 1 < garis.size(); ++i) {
        for (size_t j = i + 1; j < garis.size(); ++j) {
          double jarak = garis[j].x - garis[i].x;
          double beda  = std::fabs(jarak - harap);
          if (beda < beda_terbaik) {
            beda_terbaik = beda;
            bi = static_cast<int>(i);
            bj = static_cast<int>(j);
          }
        }
      }

      // Terima hanya kalau jaraknya masuk akal sebagai satu lajur
      if (bi >= 0 && beda_terbaik < 0.45 * harap) {
        idx_kiri  = bi;
        idx_kanan = bj;
        target_x  = (garis[bi].x + garis[bj].x) / 2.0;
        lebar_terukur_ = (garis[bj].x - garis[bi].x) / 2.0;
        return true;
      }
    }

    // --- Hanya satu garis yang bisa dipakai ---
    // Pilih yang paling dekat dengan target sebelumnya, lalu tentukan
    // sisinya dari kesinambungan itu. Memakai tengah gambar sebagai acuan
    // justru salah di tikungan tajam.
    double acuan = has_last_target_ ? last_target_x_ : w / 2.0;

    int terdekat = 0;
    double beda_terdekat = 1e9;
    for (size_t i = 0; i < garis.size(); ++i) {
      double beda = std::fabs(garis[i].x - acuan);
      if (beda < beda_terdekat) { beda_terdekat = beda; terdekat = static_cast<int>(i); }
    }

    if (garis[terdekat].x < acuan) {
      idx_kiri = terdekat;
      target_x = garis[terdekat].x + setengah;
    } else {
      idx_kanan = terdekat;
      target_x = garis[terdekat].x - setengah;
    }

    target_x = std::clamp(target_x, 0.0, static_cast<double>(w));
    return true;
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
    std::vector<Garis> garis;
    int idx_kiri = -1, idx_kanan = -1;

    bool found = cariTargetLajur(bev, band_top, band_bot, target_x,
                                 garis, idx_kiri, idx_kanan);

    bool valid = found;
    if (found) {
      // Haluskan sedikit supaya steering tidak menyentak saat jumlah
      // garis yang terlihat berubah
      if (has_last_target_) {
        target_x = 0.6 * target_x + 0.4 * last_target_x_;
      }
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

    cv::line(bev_color, cv::Point(w / 2, 0), cv::Point(w / 2, h),
             cv::Scalar(255, 0, 0), 1);

    // Semua garis yang ditemukan. Yang terpakai sebagai batas lajur
    // digambar tebal, yang lain tipis dan abu-abu.
    for (size_t i = 0; i < garis.size(); ++i) {
      int x = static_cast<int>(garis[i].x);
      bool terpakai = (static_cast<int>(i) == idx_kiri ||
                       static_cast<int>(i) == idx_kanan);

      cv::Scalar warna = !terpakai            ? cv::Scalar(120, 120, 120)
                       : (static_cast<int>(i) == idx_kiri)
                                              ? cv::Scalar(255, 128, 0)
                                              : cv::Scalar(0, 128, 255);

      cv::line(bev_color, cv::Point(x, band_top), cv::Point(x, band_bot),
               warna, terpakai ? 3 : 1);

      if (terpakai) {
        cv::putText(bev_color,
                    cv::format("%s %.0f%%", garis[i].solid ? "SOLID" : "PUTUS",
                               garis[i].isi * 100.0),
                    cv::Point(std::clamp(x - 40, 2, w - 90), band_top - 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, warna, 1);
      }
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

    const char * sumber =
        (idx_kiri >= 0 && idx_kanan >= 0) ? "2 garis"
      : (idx_kiri >= 0)                   ? "kiri saja"
      : (idx_kanan >= 0)                  ? "kanan saja"
                                          : "tidak ada";

    cv::putText(bev_color,
                cv::format("sumber: %s  (%zu kandidat)", sumber, garis.size()),
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