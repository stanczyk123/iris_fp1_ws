// IRIS FP1 - Vision Node
// Step 4: Image Thresholding & Bird-Eye View (BEV)
// Step 5: Finding Turn Target
//
// Alur: kamera -> threshold HSV -> warp BEV -> cari titik belok
// Hasil dipublish ke /turn_target (geometry_msgs/Point):
//   x = posisi target (piksel, 0..lebar BEV)
//   y = baris lookahead (piksel)
//   z = 1.0 kalau target valid, 0.0 kalau pakai nilai terakhir
//
// Tekan 's' untuk print parameter, 'q' untuk keluar.

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
  int look_y;      // posisi pita pencarian, % dari tinggi BEV
  int band_h;      // tebal pita, % dari tinggi BEV
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
    cv::resizeWindow(WIN_TUNING, 460, 400);

    // Nilai hasil tuning
    p_.h_min = 0;   p_.h_max = 179;
    p_.s_min = 0;   p_.s_max = 40;
    p_.v_min = 180; p_.v_max = 255;

    p_.top_w = 20;  p_.bot_w = 95;
    p_.top_y = 60;  p_.bot_y = 95;

    p_.look_y = 50;   // pita di tengah gambar BEV
    p_.band_h = 15;

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
  // Step 5: cari titik belok dari gambar BEV
  //
  // Ambil satu pita horizontal pada jarak lookahead, lalu hitung titik
  // berat (centroid) piksel putih di pita itu. Karena garis tengah
  // berupa garis putus-putus, pita bisa kosong sesaat -- kalau begitu
  // kita pakai target terakhir yang valid supaya mobil tidak menyentak.
  // ----------------------------------------------------------------
  bool findTurnTarget(const cv::Mat & bev, int & band_top, int & band_bot,
                      double & target_x)
  {
    const int h = bev.rows;

    int band_center = static_cast<int>(p_.look_y / 100.0 * h);
    int half_band   = std::max(1, static_cast<int>(p_.band_h / 100.0 * h / 2));

    band_top = std::max(0, band_center - half_band);
    band_bot = std::min(h, band_center + half_band);

    if (band_bot <= band_top) return false;

    cv::Mat band = bev(cv::Range(band_top, band_bot), cv::Range::all());

    // Momen orde nol = jumlah piksel putih, orde satu = jumlah posisi x
    cv::Moments m = cv::moments(band, true);

    if (m.m00 < 50) {       // terlalu sedikit piksel, anggap tidak ketemu
      return false;
    }

    target_x = m.m10 / m.m00;
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

    // --- Cari titik belok ---
    int band_top = 0, band_bot = 0;
    double target_x = 0.0;
    bool found = findTurnTarget(bev, band_top, band_bot, target_x);

    bool valid = found;
    if (found) {
      last_target_x_   = target_x;
      has_last_target_ = true;
    } else if (has_last_target_) {
      target_x = last_target_x_;   // pakai nilai terakhir saat garis putus
    } else {
      target_x = w / 2.0;          // belum pernah ketemu: anggap lurus
    }

    int band_center_y = (band_top + band_bot) / 2;

    auto point = geometry_msgs::msg::Point();
    point.x = target_x;
    point.y = static_cast<double>(band_center_y);
    point.z = valid ? 1.0 : 0.0;
    pub_->publish(point);

    // --- Tampilan ---
    cv::Mat bev_color;
    cv::cvtColor(bev, bev_color, cv::COLOR_GRAY2BGR);

    // Pita pencarian (kuning)
    cv::rectangle(bev_color, cv::Point(0, band_top), cv::Point(w, band_bot),
                  cv::Scalar(0, 255, 255), 2);

    // Garis tengah gambar = arah lurus (biru)
    cv::line(bev_color, cv::Point(w / 2, 0), cv::Point(w / 2, h),
             cv::Scalar(255, 0, 0), 1);

    // Titik target (hijau kalau valid, merah kalau pakai nilai terakhir)
    cv::Scalar color = valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::circle(bev_color, cv::Point(static_cast<int>(target_x), band_center_y),
               10, color, -1);
    cv::line(bev_color, cv::Point(w / 2, h),
             cv::Point(static_cast<int>(target_x), band_center_y), color, 2);

    double offset = target_x - w / 2.0;
    cv::putText(bev_color,
                cv::format("offset: %+.1f px", offset),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 255), 2);

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
                  "look_y=%d band_h=%d",
                  p_.h_min, p_.s_min, p_.v_min,
                  p_.h_max, p_.s_max, p_.v_max,
                  p_.top_w, p_.bot_w, p_.top_y, p_.bot_y,
                  p_.look_y, p_.band_h);
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionNode>());
  rclcpp::shutdown();
  return 0;
}