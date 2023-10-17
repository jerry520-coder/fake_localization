#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class FakeLocalization : public rclcpp::Node {
public:
    FakeLocalization() : Node("fake_localization") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10, std::bind(&FakeLocalization::odomCallback, this, std::placeholders::_1));
        amcl_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10);
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "initialpose", 10, std::bind(&FakeLocalization::initPoseCallback, this, std::placeholders::_1));
    }

private:
    void initPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        initial_pose_ = *msg;
        has_initial_pose_ = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 如果提供了初始位姿，则使用初始位姿和odom数据来计算amcl_pose和TF变换
        geometry_msgs::msg::Pose pose;
        if (has_initial_pose_) {
            // 这里简单地将初始位姿和odom数据相加，但实际应用中可能需要更复杂的处理
            pose.position.x = initial_pose_.pose.pose.position.x + msg->pose.pose.position.x;
            pose.position.y = initial_pose_.pose.pose.position.y + msg->pose.pose.position.y;
            pose.position.z = initial_pose_.pose.pose.position.z + msg->pose.pose.position.z;
            pose.orientation = initial_pose_.pose.pose.orientation;// 这是一个简化的处理，实际应用可能需要更复杂的方向计算
        } else {
            pose = msg->pose.pose;
        }

        // 发布amcl_pose消息
        geometry_msgs::msg::PoseWithCovarianceStamped amcl_pose;
        amcl_pose.header.stamp = msg->header.stamp;
        amcl_pose.header.frame_id = "map";
        amcl_pose.pose.pose = pose;
        amcl_pose_pub_->publish(amcl_pose);

        // odom->base_link的TF变换
        tf2::Transform transform;
        transform.setOrigin({pose.position.x, pose.position.y, pose.position.z});
        transform.setRotation({pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w});
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.transform = tf2::toMsg(transform);
        transform_stamped.header.stamp =  msg->header.stamp;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_link";
        broadcaster_->sendTransform(transform_stamped);
        // map->odom的TF变换
       
        transform.setOrigin({0, 0, 0});
        transform.setRotation({0, 0, 0, 1});
        transform_stamped.transform = tf2::toMsg(transform);
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "odom";
        broadcaster_->sendTransform(transform_stamped);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_;
    bool has_initial_pose_ = false;
    rclcpp::Clock::SharedPtr clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeLocalization>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
