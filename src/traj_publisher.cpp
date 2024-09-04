// File: src/tf_to_path_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include <fstream>

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
        this->declare_parameter<std::string>("csv_file_name", "path_output.csv");
        this->get_parameter("target_frame", target_frame_);
        this->get_parameter("origin_frame", origin_frame_);
        this->get_parameter("csv_file_name", csv_file_name_);

        // Open the CSV file for writing
        csv_file_.open(csv_file_name_);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_file_name_.c_str());
        } else {
            // Write the header to the CSV file
            csv_file_ << "timestamp,x,y,z,qx,qy,qz,qw\n";
        }

        // Publisher for the Path message
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 100);

        // Initialize the Path message header
        path_msg_->header.frame_id = origin_frame_;

        // Create a timer to periodically check and publish the path
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TrajPubNode::timer_callback, this));
    }


    ~TrajPubNode() {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
        RCLCPP_INFO(this->get_logger(), "CSV file has been written to: %s", csv_file_name_.c_str());
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

                        // Append pose data to the CSV file
            if (csv_file_.is_open()) {
                csv_file_ << pose_stamped.header.stamp.sec << "."
                          << pose_stamped.header.stamp.nanosec << ","
                          << pose_stamped.pose.position.x << ","
                          << pose_stamped.pose.position.y << ","
                          << pose_stamped.pose.position.z << ","
                          << pose_stamped.pose.orientation.x << ","
                          << pose_stamped.pose.orientation.y << ","
                          << pose_stamped.pose.orientation.z << ","
                          << pose_stamped.pose.orientation.w << "\n";
            }
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

    std::string csv_file_name_;
    std::ofstream csv_file_;

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
