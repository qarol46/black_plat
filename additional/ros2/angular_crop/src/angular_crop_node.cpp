#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <vector>

class AngularCropNode : public rclcpp::Node
{
public:
  AngularCropNode() : Node("angular_crop")
  {
    // Азимут задаётся в градусах, угол отсчитывается от оси X по часовой стрелке
    // при виде сверху. Диапазон [-180, 180].
    // Пример: angle_min=-90, angle_max=90 оставит переднюю полусферу.
    this->declare_parameter("input_topic",  "/velodyne_points");
    this->declare_parameter("output_topic", "/velodyne_points/cropped");
    this->declare_parameter("angle_min_deg", -90.0);
    this->declare_parameter("angle_max_deg",  90.0);

    const auto in  = this->get_parameter("input_topic").as_string();
    const auto out = this->get_parameter("output_topic").as_string();
    angle_min_ = this->get_parameter("angle_min_deg").as_double() * M_PI / 180.0;
    angle_max_ = this->get_parameter("angle_max_deg").as_double() * M_PI / 180.0;

    rclcpp::QoS qos(1);
    qos.best_effort();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      in, qos,
      std::bind(&AngularCropNode::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out, 10);

    RCLCPP_INFO(get_logger(),
      "angular_crop: %s → %s  [%.1f°, %.1f°]",
      in.c_str(), out.c_str(),
      angle_min_ * 180.0 / M_PI,
      angle_max_ * 180.0 / M_PI);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // ── Итераторы для чтения x, y ────────────────────────────────────────
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");

    const uint32_t point_step = msg->point_step;
    const uint8_t* data_ptr   = msg->data.data();
    const size_t   n_points   = msg->width * msg->height;

    // ── Маска индексов точек, прошедших фильтр ───────────────────────────
    std::vector<uint32_t> indices;
    indices.reserve(n_points);

    const bool wraps = angle_min_ > angle_max_; // диапазон пересекает ±180°

    for (size_t i = 0; i < n_points; ++i, ++iter_x, ++iter_y) {
      const float x = *iter_x;
      const float y = *iter_y;

      // NaN и нулевые точки пропускаем
      if (!std::isfinite(x) || !std::isfinite(y)) continue;

      const double az = std::atan2(y, x); // [-π, π]

      bool inside;
      if (!wraps) {
        inside = (az >= angle_min_) && (az <= angle_max_);
      } else {
        // Диапазон вида [170°, -170°] — пересекает разрыв ±180°
        inside = (az >= angle_min_) || (az <= angle_max_);
      }

      if (inside) {
        indices.push_back(static_cast<uint32_t>(i));
      }
    }

    // ── Собираем выходное сообщение ──────────────────────────────────────
    auto out_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out_msg->header     = msg->header;
    out_msg->fields     = msg->fields;
    out_msg->point_step = point_step;
    out_msg->height     = 1;
    out_msg->width      = static_cast<uint32_t>(indices.size());
    out_msg->row_step   = point_step * out_msg->width;
    out_msg->is_dense   = false;
    out_msg->is_bigendian = msg->is_bigendian;

    out_msg->data.resize(out_msg->row_step);
    uint8_t* dst = out_msg->data.data();

    for (const uint32_t idx : indices) {
      std::memcpy(dst, data_ptr + idx * point_step, point_step);
      dst += point_step;
    }

    pub_->publish(std::move(out_msg));
  }

  double angle_min_;
  double angle_max_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngularCropNode>());
  rclcpp::shutdown();
  return 0;
}