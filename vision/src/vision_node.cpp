// IRIS FP1 - Vision Node
// Step 4: Image Thresholding & Bird-Eye View (BEV)
//
// Subscribe kamera simulasi -> threshold HSV -> warp ke Bird-Eye View.
// Parameter diatur lewat slider di window "Tuning".
// Tekan 's' untuk print nilai parameter saat ini, 'q' untuk keluar.

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>

namespace
{
const std::string CAMERA_TOPIC = "/catvehicle/camera_front/image_raw_front";

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
};

class VisionNode : public rclcpp::Node
{
public:
  VisionNode() : Node("vision_node"), frame_count_(0)
  {
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      CAMERA_TOPIC, rclcpp::SensorDataQoS(),
      std::bind(&VisionNode::imageCallback, this, std::placeholders::_1));

    buildTuningWindow();

    RCLCPP_INFO(this->get_logger(), "Vision node aktif. Subscribe: %s",
                CAMERA_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "Menunggu gambar pertama...");
  }

  ~VisionNode() { cv::destroyAllWindows(); }

private:
  void buildTuningWindow()
  {
    cv::namedWindow(WIN_TUNING, cv::WINDOW_NORMAL);
    cv::resizeWindow(WIN_TUNING, 460, 340);

    // Default diarahkan ke warna putih: saturasi rendah, value tinggi
    p_.h_min = 0;   p_.h_max = 179;
    p_.s_min = 0;   p_.s_max = 40;
    p_.v_min = 180; p_.v_max = 255;

    p_.top_w = 20;  p_.bot_w = 95;
    p_.top_y = 60;  p_.bot_y = 95;

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
      cv::Point2f(cx - (top_w * w / 2.0f), top_y),  // kiri atas
      cv::Point2f(cx + (top_w * w / 2.0f), top_y),  // kanan atas
      cv::Point2f(cx + (bot_w * w / 2.0f), bot_y),  // kanan bawah
      cv::Point2f(cx - (bot_w * w / 2.0f), bot_y)   // kiri bawah
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

    // Gambar trapesium di frame asli supaya kelihatan area mana yang diambil
    cv::Mat preview = frame.clone();
    std::vector<cv::Point> poly;
    for (const auto & pt : src) {
      poly.emplace_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
    }
    cv::polylines(preview, poly, true, cv::Scalar(0, 0, 255), 2);

    cv::imshow(WIN_RAW, preview);
    cv::imshow(WIN_THRESH, mask);
    cv::imshow(WIN_BEV, bev);

    int key = cv::waitKey(1) & 0xFF;
    if (key == 's') {
      RCLCPP_INFO(this->get_logger(),
                  "HSV: [%d,%d,%d] - [%d,%d,%d] | BEV: atas_w=%d bawah_w=%d "
                  "atas_y=%d bawah_y=%d",
                  p_.h_min, p_.s_min, p_.v_min,
                  p_.h_max, p_.s_max, p_.v_max,
                  p_.top_w, p_.bot_w, p_.top_y, p_.bot_y);
    } else if (key == 'q') {
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  TuningParams p_;
  unsigned long frame_count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionNode>());
  rclcpp::shutdown();
  return 0;
}