/**
 * Based on https://github.com/czw90130/virtual_camera/. It will:
 *
 * - subscribe to a sensor_msgs::Image topic
 * - feed received images to an ImageConverter.
 * - write the converted image to a VideoDevice.
 *
 * \copyright Copyright (c) 2013, Zhiwei Chu
 * \copyright Copyright (c) 2015, mayfieldrobotics.
 * \license This project is released under the BSD License.
 *
 */

#include <image_to_v4l2loopback/image_converter.h>
#include <image_to_v4l2loopback/video_device.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.h>
#include <string>

class ImageStreamNode : public rclcpp::Node {
public:
  ImageStreamNode() : Node("image_stream_node")
  {
    // Declare and get parameters
    this->declare_parameter<std::string>("topic", "image");
    std::string topic = this->get_parameter("topic").as_string();

    this->declare_parameter<int>("width", 640);
    this->declare_parameter<int>("height", 480);
    const uint32_t width = this->get_parameter("width").as_int();    
    const uint32_t height = this->get_parameter("height").as_int();


    // valid options are BGR3, RGB3, GREY, YV12, YUYV
    this->declare_parameter<std::string>("format", "YV12");
    std::string format = this->get_parameter("format").as_string();

    this->declare_parameter<std::string>("device", "/dev/video1");
    std::string video_device = this->get_parameter("device").as_string();

    this->declare_parameter<int>("queue_size", 1);
    int queue_size = this->get_parameter("queue_size").as_int();

    RCLCPP_INFO(this->get_logger(), "Converting - width=%d, height=%d, format=%s", width, height, format.c_str());
    image_converter_ = std::make_unique<ImageConverter>(width, height, format, this->get_logger());

    RCLCPP_INFO(this->get_logger(), "opening '%s'", video_device.c_str());
    video_device_ = std::make_unique<VideoDevice>(video_device, this->get_logger());

    /**
     * Writes converted images from a sensor_msgs::Image to a VideoDevice.
     */
    auto callback = [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
      ImageConverter::Buffer buf;
      int rc = image_converter_->convert(msg, buf);
      if (rc == -1) {
        return;
      }

      rc = video_device_->write(buf.data(), buf.size());
      if (rc == -1) {
        return;
      }
    };

    // image_transport::ImageTransport it(this->shared_from_this());
    sub_ = image_transport::create_subscription(this, topic, callback, "raw", rmw_qos_profile_sensor_data);
    int rc;

    v4l2_capability capability;
    rc = video_device_->capabilities(capability);
    if (rc == -1) {
      throw std::runtime_error("failed device caps");
    }
    RCLCPP_INFO(this->get_logger(), "'%s' caps %#08x", video_device.c_str(), capability.capabilities);

    v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    rc = video_device_->get_format(fmt);
    if (rc == -1) {
      throw std::runtime_error("failed to get device format");
    }
    video_device_->log_format("current format: ", fmt);

    RCLCPP_INFO(this->get_logger(), "setting '%s' format", video_device.c_str());
    fmt = image_converter_->format();
    rc = video_device_->set_format(fmt);
    if (rc == -1) {
      throw std::runtime_error("failed device format: " + format);
    }

    RCLCPP_INFO(this->get_logger(), "turn on '%s' streaming", video_device.c_str());
    rc = video_device_->stream_on();
    if (rc == -1) {
      throw std::runtime_error("failed device stream");
    }

    RCLCPP_INFO(this->get_logger(), "streaming images from '%s' to '%s' w/ queue-size=%u",
             topic.c_str(), video_device.c_str(),
             queue_size);
  }

private:
  std::unique_ptr<ImageConverter> image_converter_;
  std::unique_ptr<VideoDevice> video_device_;
  image_transport::Subscriber sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageStreamNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}