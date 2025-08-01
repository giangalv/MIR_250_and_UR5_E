#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer()
        : Node("pointcloud_left_transformer"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        this->declare_parameter<std::string>("input_topic", "/camera_floor_left/obstacles");
        this->declare_parameter<std::string>("output_topic", "/camera_floor_left/transformed_obstacles");
        this->declare_parameter<std::string>("target_frame", "base_link");

        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&PointCloudTransformer::cloud_callback, this, _1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "Subscribed to %s, publishing to %s in frame %s",
                    input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str());
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (msg->header.frame_id == target_frame_)
        {
            pub_->publish(*msg);
            return;
        }

        try
        {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                target_frame_, msg->header.frame_id, msg->header.stamp,
                rclcpp::Duration::from_seconds(0.2));

            sensor_msgs::msg::PointCloud2 transformed_cloud;
            tf2::doTransform(*msg, transformed_cloud, transform);
            pub_->publish(transformed_cloud);
        }
        catch (const tf2::TransformException &ex)
        {
            return;
            //RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        }
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string target_frame_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
