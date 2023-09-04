#include "rclcpp/rclcpp.hpp"
#include "object_detection_service/srv/object_detection.hpp"  // Update with your package name and service
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class ObjectDetectionServer : public rclcpp::Node
{
public:
  ObjectDetectionServer()
  : Node("object_detection_server")
  {
    service_ = this->create_service<object_detection_service::srv::ObjectDetection>(
      "object_detection_service",
      std::bind(&ObjectDetectionServer::handle_service, this, _1, _2)
    );
    // Initialize YOLOv3 model
    net = cv::dnn::readNet("/path/to/yolov3.weights", "/path/to/yolov3.cfg");
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  }

  void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<object_detection_service::srv::ObjectDetection::Request> req,
    const std::shared_ptr<object_detection_service::srv::ObjectDetection::Response> res)
  {
    // Convert ROS Image to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(req->input_image, "bgr8");

    // Run YOLOv3 model
    cv::Mat& frame = cv_ptr->image;

    // Convert frame to blob
    cv::Mat blob = cv::dnn::blobFromImage(frame, 1 / 255.0, cv::Size(416, 416), cv::Scalar(0, 0, 0), true, false);
    net.setInput(blob);

    // Run forward pass to get output
    std::vector<cv::Mat> out;
    net.forward(out, getOutputsNames(net));

    // Process the output
    std::vector<std::string> object_names;
    for (auto &prob : out) {
      processFrame(prob, frame, object_names);
    }

    // Set the response
    res->object_names = object_names;
  }

private:
  std::vector<std::string> getOutputsNames(const cv::dnn::Net& net)
  {
    static std::vector<std::string> names;
    if (names.empty()) {
      std::vector<int> outLayers = net.getUnconnectedOutLayers();
      std::vector<std::string> layersNames = net.getLayerNames();
      names.resize(outLayers.size());
      for (size_t i = 0; i < outLayers.size(); ++i) {
        names[i] = layersNames[outLayers[i] - 1];
      }
    }
    return names;
  }

  void processFrame(cv::Mat& prob, cv::Mat& frame, std::vector<std::string>& object_names)
  {
    // Your code to process the output and extract object names
    // This is specific to your object detection model and classes
  }

  rclcpp::Service<object_detection_service::srv::ObjectDetection>::SharedPtr service_;
  cv::dnn::Net net;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetectionServer>());
  rclcpp::shutdown();
  return 0;
}
