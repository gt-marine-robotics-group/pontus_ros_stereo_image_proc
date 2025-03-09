#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp> 
#include <opencv2/cudawarping.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/cudastereo.hpp>

class DisparityNode : public rclcpp::Node {
public:
    DisparityNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("disparity_node", options) {

        // Ensure correct QoS for sensor messages
        left_image_subscriber.subscribe(this, "/left/image_rect", rmw_qos_profile_sensor_data);
        right_image_subscriber.subscribe(this, "/right/image_rect", rmw_qos_profile_sensor_data);

        sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>(
            message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>(10),
            left_image_subscriber, right_image_subscriber
        );
        sync->setAgePenalty(0.5);
        sync->registerCallback(std::bind(&DisparityNode::camera_callback, this, std::placeholders::_1, std::placeholders::_2));

        disparity_pubisher = this->create_publisher<sensor_msgs::msg::Image>(
            "/disparity_image",
            10
        );

        // Parameters
        this->declare_parameter<int>("prefilter_cap", 31);
        this->declare_parameter<int>("texture_threshold", 10);
        this->declare_parameter<int>("uniqueness_ratio", 15);
        this->declare_parameter<int>("speckle_window_size", 100);
        this->declare_parameter<int>("speckle_range", 16);
        this->declare_parameter<int>("window_size", 9);
        this->declare_parameter<int>("num_disparities", 32);


        this->get_parameter("prefilter_cap", prefilter_cap);
        this->get_parameter("texture_threshold", texture_threshold);
        this->get_parameter("uniqueness_ratio", uniqueness_ratio);
        this->get_parameter("speckle_window_size", speckle_window_size);
        this->get_parameter("speckle_range", speckle_range);
        this->get_parameter("window_size", window_size);
        this->get_parameter("num_disparities", num_disparities);
    }

private:
    void camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr &left_image, const sensor_msgs::msg::Image::ConstSharedPtr &right_image) {
        cv::Mat left_img = cv_bridge::toCvCopy(left_image, "mono8")->image;
        cv::Mat right_img = cv_bridge::toCvCopy(right_image, "mono8")->image;

        cv::cuda::GpuMat d_left, d_right, d_disparity;
        d_left.upload(left_img);
        d_right.upload(right_img);

        auto stereoBM = cv::cuda::createStereoBM(num_disparities, window_size);
        stereoBM->setPreFilterCap(prefilter_cap);
        stereoBM->setTextureThreshold(texture_threshold);
        stereoBM->setUniquenessRatio(uniqueness_ratio);
        stereoBM->setSpeckleWindowSize(speckle_window_size);
        stereoBM->setSpeckleRange(speckle_range);
        stereoBM->compute(d_left, d_right, d_disparity);
        cv::Mat disparity;
        d_disparity.download(disparity);
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "mono8",
            disparity).toImageMsg();
        disparity_pubisher->publish(*msg);
    }
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pubisher;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_image_subscriber;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_image_subscriber;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync;
    int prefilter_cap;
    int texture_threshold;
    int uniqueness_ratio;
    int speckle_window_size;
    int speckle_range;
    int window_size;
    int num_disparities;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DisparityNode>(rclcpp::NodeOptions().use_intra_process_comms(true));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
