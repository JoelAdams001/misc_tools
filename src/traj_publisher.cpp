// File: src/tf_to_path_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

class TrajPubNode : public rclcpp::Node
{
public:
    TrajPubNode() : Node("traj_publisher"), 
                     path_msg_(new nav_msgs::msg::Path()),
                     tf_buffer_(this->get_clock()), 
                     tf_listener_(tf_buffer_)
    {
        // Declare and get parameters for the target frame and origin frame
        this->declare_parameter<std::string>("target_frame", "base_link");
        this->declare_parameter<std::string>("origin_frame", "map");
        this->get_parameter("target_frame", target_frame_);
        this->get_parameter("origin_frame", origin_frame_);

        // Publisher for the Path message
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

        // Initialize the Path message header
        path_msg_->header.frame_id = origin_frame_;

        // Create a timer to periodically check and publish the path
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrajPubNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        try
        {
            // Lookup the transform from the origin frame to the target frame
            geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_.lookupTransform(
                origin_frame_, target_frame_, tf2::TimePointZero);

            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = this->get_clock()->now();
            pose_stamped.header.frame_id = origin_frame_;
            pose_stamped.pose.position.x = tf_stamped.transform.translation.x;
            pose_stamped.pose.position.y = tf_stamped.transform.translation.y;
            pose_stamped.pose.position.z = tf_stamped.transform.translation.z;
            pose_stamped.pose.orientation = tf_stamped.transform.rotation;

            // Add the pose to the path
            path_msg_->poses.push_back(pose_stamped);

            // Update the path message header timestamp
            path_msg_->header.stamp = this->get_clock()->now();

            // Publish the path
            path_publisher_->publish(*path_msg_);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                        target_frame_.c_str(), origin_frame_.c_str(), ex.what());
        }
    }

    std::string target_frame_;
    std::string origin_frame_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    nav_msgs::msg::Path::SharedPtr path_msg_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajPubNode>());
    rclcpp::shutdown();
    return 0;
}
