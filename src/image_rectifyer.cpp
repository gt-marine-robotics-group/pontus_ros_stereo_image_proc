#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp> 
#include <opencv2/cudawarping.hpp>

class ImageRectifyer : public rclcpp::Node {
public:
    ImageRectifyer(): Node("image_rectifier") {
        image_subscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            10,
            std::bind(&ImageRectifyer::camera_callback, this, std::placeholders::_1)
        );
        camera_info_subscription = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info",
            10,
            std::bind(&ImageRectifyer::camera_info_callback, this, std::placeholders::_1)
        );
        image_rect_color_publisher = this->create_publisher<sensor_msgs::msg::Image>(
            "/image_rect_color",
            10
        );
        image_rect_publisher = this->create_publisher<sensor_msgs::msg::Image>(
            "/image_rect",
            10
        );
    }
private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
        dist_coeffs = cv::Mat(1, 5, CV_64F, const_cast<double*>(msg->d.data())).clone();
    }

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
        // Camera intrinsics are needed for rectification
        if (camera_matrix.empty() || dist_coeffs.empty()) {
            RCLCPP_WARN(this->get_logger(), "Camera intrinsics are not yet available.");
            return;
        }
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        cv::cuda::GpuMat gpu_undisorted;
        // Upload image to gpu
        cv::cuda::GpuMat gpu_image(cv_ptr->image);
        cv::Mat map1, map2;
        cv::Size image_size(gpu_image.cols, gpu_image.rows);
        // Init map
        cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), camera_matrix, image_size, CV_32F, map1, map2);
        cv::cuda::GpuMat gpu_map1(map1), gpu_map2(map2);

        // Perform rectification
        cv::cuda::remap(gpu_image, gpu_undisorted, gpu_map1, gpu_map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

        // Get image from gpu
        cv::Mat undistorted_image;
        gpu_undisorted.download(undistorted_image);

        // Convert to gray scale
        cv::Mat grayscale_image;
        cv::cvtColor(undistorted_image, grayscale_image, cv::COLOR_BGR2GRAY);
        
        // Publish to /image_rect_color
        cv_bridge::CvImagePtr out_cv_ptr_color = std::make_shared<cv_bridge::CvImage>();
        out_cv_ptr_color->header = msg->header;
        out_cv_ptr_color->encoding = sensor_msgs::image_encodings::BGR8;
        out_cv_ptr_color->image = undistorted_image;
        
        image_rect_color_publisher->publish(std::make_unique<sensor_msgs::msg::Image>(*out_cv_ptr_color->toImageMsg()));

        // Publish to /image_rect
        cv_bridge::CvImagePtr out_cv_ptr = std::make_shared<cv_bridge::CvImage>();
        out_cv_ptr->header = msg->header;
        out_cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
        out_cv_ptr->image = grayscale_image;
        
        image_rect_publisher->publish(std::make_unique<sensor_msgs::msg::Image>(*out_cv_ptr->toImageMsg()));
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_rect_color_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_rect_publisher;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageRectifyer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}