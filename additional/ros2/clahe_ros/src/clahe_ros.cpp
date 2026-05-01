#include "clahe_ros/clahe_ros.h"

// Строим LUT для гамма-коррекции один раз при изменении параметра
static cv::Mat buildGammaLUT(double gamma)
{
  cv::Mat lut(1, 256, CV_8U);
  for (int i = 0; i < 256; ++i) {
    lut.at<uint8_t>(i) =
      cv::saturate_cast<uint8_t>(std::pow(i / 255.0, gamma) * 255.0);
  }
  return lut;
}

ClaheRos::ClaheRos(const rclcpp::NodeOptions & options)
: Node("clahe_ros", options)
{
  clahe_clip_limit_sky_ = declare_parameter<double>("clahe_clip_limit_sky", 2.5);
  clahe_grid_size_sky_  = declare_parameter<int>("clahe_grid_size_sky", 15);
  clahe_clip_limit_asphalt_ = declare_parameter<double>("clahe_clip_limit_asphalt", 2.5);
  clahe_grid_size_asphalt_  = declare_parameter<int>("clahe_grid_size_asphalt", 15);
  rect_sky_x_ = declare_parameter<int>("rect_sky_x", 70);
  rect_sky_y_ = declare_parameter<int>("rect_sky_y", 0);
  rect_sky_w_ = declare_parameter<int>("rect_sky_w", 500);
  rect_sky_h_ = declare_parameter<int>("rect_sky_h", 150);
  rect_asphalt_x_ = declare_parameter<int>("rect_asphalt_x", 0);
  rect_asphalt_y_ = declare_parameter<int>("rect_asphalt_y", 380);
  rect_asphalt_w_ = declare_parameter<int>("rect_asphalt_w", 640);
  rect_asphalt_h_ = declare_parameter<int>("rect_asphalt_h", 200);
  // γ < 1 → сжимает яркие области, поднимает тёмные
  // γ = 1 → без изменений (отключить гамму)
  // Рекомендуемый диапазон для засвеченного IR: 0.4 – 0.7
  gamma_            = declare_parameter<double>("gamma", 0.45);

  auto node = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

  image_pub_ = image_transport::create_publisher(
    node.get(), "image/filtered");

  image_sub_ = image_transport::create_subscription(
    node.get(),
    "image/raw",
    std::bind(&ClaheRos::imageCb, this, std::placeholders::_1),
    "raw");
}

void ClaheRos::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, "mono8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat processed = cv_ptr->image.clone();

  // ── Шаг 1: Гамма-коррекция ────────────────────────────────────────────────
  // Применяем только если gamma != 1.0
  if (std::abs(gamma_ - 1.0) > 1e-3) {
    cv::Mat lut = buildGammaLUT(gamma_);
    cv::LUT(processed, lut, processed);
  }

  // ── Шаг 2: CLAHE для локального усиления контраста неба ────────────────────────
  // После гаммы изображение стало «ровнее», CLAHE вытягивает детали дороги
  if (clahe_grid_size_sky_ > 1) {
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(clahe_clip_limit_sky_);
    clahe->setTilesGridSize(cv::Size(clahe_grid_size_sky_, clahe_grid_size_sky_));
    // 1. Define your area: x, y, width, height
    // You can make these ROS parameters so you can tune them live via rqt
    cv::Rect sky_rect(rect_sky_x_, rect_sky_y_, rect_sky_w_, rect_sky_h_);
    // 2. Safety Check: Ensure the rect is strictly inside the image bounds
    // The bitwise AND operator creates a safe intersection
    cv::Rect image_bounds(0, 0, processed.cols, processed.rows);
    cv::Rect safe_sky = sky_rect & image_bounds; 
    if (safe_sky.area() > 0) {
    // 3. Create the ROI reference (No memory allocation happens here)
    cv::Mat sky_roi = processed(safe_sky);
    // 4. Apply CLAHE only to that specific memory block
    // The original 'processed' matrix is updated automatically
    clahe->apply(sky_roi, sky_roi);
    }
  }
  // ── Шаг 3: CLAHE для локального усиления контраста асфальта ────────────────────────
  // После гаммы изображение стало «ровнее», CLAHE вытягивает детали дороги
  if (clahe_grid_size_asphalt_ > 1) {
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(clahe_clip_limit_asphalt_);
    clahe->setTilesGridSize(cv::Size(clahe_grid_size_asphalt_, clahe_grid_size_asphalt_));
    // 1. Define your area: x, y, width, height
    // You can make these ROS parameters so you can tune them live via rqt
    cv::Rect asphalt_rect(rect_asphalt_x_, rect_asphalt_y_, rect_asphalt_w_, rect_asphalt_h_);
    // 2. Safety Check: Ensure the rect is strictly inside the image bounds
    // The bitwise AND operator creates a safe intersection
    cv::Rect image_bounds(0, 0, processed.cols, processed.rows);
    cv::Rect safe_asphalt = asphalt_rect & image_bounds; 
    if (safe_asphalt.area() > 0) {
    // 3. Create the ROI reference (No memory allocation happens here)
    cv::Mat asphalt_roi = processed(safe_asphalt);
    // 4. Apply CLAHE only to that specific memory block
    // The original 'processed' matrix is updated automatically
    clahe->apply(asphalt_roi, asphalt_roi);
    }
  }
  
  // ── Шаг 4: Растяжение контраста ───────────────────────────────────────────
  // Находит min/max в кадре и растягивает диапазон на 0–255
  // Эффективно после гаммы — убирает «серость» сжатого изображения
  cv::normalize(processed, processed, 0, 255, cv::NORM_MINMAX, CV_8U);
  
  cv_bridge::CvImage out_msg;
  out_msg.header   = msg->header;
  out_msg.encoding = "mono8";
  out_msg.image    = processed;

  image_pub_.publish(out_msg.toImageMsg());
}