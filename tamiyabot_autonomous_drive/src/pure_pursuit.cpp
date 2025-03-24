#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

double to_radians(double theta) {
    return M_PI * theta / 180.0;
}

double to_degree(double theta) {
    return theta * 180 / M_PI;
}

class PursuitNode : public rclcpp::Node {
public:
    PursuitNode() : Node("pursuit_node") {
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, std::bind(&PursuitNode::pathCallback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/tamiyabot_controller/odometry", 10, std::bind(&PursuitNode::odomCallback, this, std::placeholders::_1));
        command_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/tamiyabot_controller/reference", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead_point", 10);

        lookahead_distance_ = this->declare_parameter("lookahead_distance", 0.4);
        max_velocity_ = this->declare_parameter("max_velocity", 1.0);
        mid_velocity_ = this->declare_parameter("mid_velocity", 0.5);
        min_velocity_ = this->declare_parameter("min_velocity", 0.2);

        RCLCPP_INFO(this->get_logger(), "Pursuit node has been started.");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    double current_position_x_ = 0.0;
    double current_position_y_ = 0.0;
    double current_heading_ = 0.0;
    double lookahead_distance_ = 0.8;
    double Lf = 0.42;
    double WB = 0.22;

    int visited_index = -1;
    nav_msgs::msg::Path::SharedPtr path_;

    double max_velocity_;
    double mid_velocity_;
    double min_velocity_;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = msg;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        current_position_x_ = msg->pose.pose.position.x;
        current_position_y_ = msg->pose.pose.position.y;
        current_heading_ = yaw;

        pursuePath();
    }

    void pursuePath() {
        if (!path_ || path_->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Path is empty or not received yet.");
            return;
        }

        if (Lf <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Lf must be greater than zero.");
            return;
        }

        auto target_pose = findLookaheadPoint();
        publish_marker_lookahead(target_pose.position.x, target_pose.position.y);

        double alpha = std::atan2((target_pose.position.y - current_position_y_), (target_pose.position.x - current_position_x_)) - current_heading_;
        double delta = std::atan2(2.0 * WB * std::sin(alpha) / Lf, 1.0);

        geometry_msgs::msg::TwistStamped command;
        command.header.stamp = this->now();
        command.header.frame_id = "base_link";
        command.twist.linear.x = get_velocity(delta);
        command.twist.angular.z = delta;

        command_publisher_->publish(command);
    }

    void publish_marker_lookahead(double _x, double _y) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "lookahead_point";
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.pose.position.x = _x;
        marker.pose.position.y = _y;
        marker.pose.position.z = 0.0;
        marker.id = 1;

        marker_publisher_->publish(marker);
    }

    geometry_msgs::msg::Pose findLookaheadPoint() {
        double dx, dy, distance;
        geometry_msgs::msg::Pose pose_;

        if (visited_index == -1) {
            for (size_t i = 0; i < path_->poses.size(); i++) {
                dx = path_->poses[i].pose.position.x - current_position_x_;
                dy = path_->poses[i].pose.position.y - current_position_y_;
                distance = std::sqrt(dx * dx + dy * dy);
                if (distance >= lookahead_distance_) {
                    visited_index = i;
                    return path_->poses[i].pose;
                }
            }
        } else {
            for (size_t i = visited_index; i < path_->poses.size(); i++) {
                dx = path_->poses[i].pose.position.x - current_position_x_;
                dy = path_->poses[i].pose.position.y - current_position_y_;
                distance = std::sqrt(dx * dx + dy * dy);
                if (distance >= lookahead_distance_) {
                    visited_index = i;
                    return path_->poses[i].pose;
                }
            }
        }

        RCLCPP_WARN(this->get_logger(), "No valid lookahead point found within the lookahead distance.");
        return geometry_msgs::msg::Pose();
    }

    double get_velocity(double steering_angle) {
        if (abs(steering_angle) >= to_radians(0.0) && abs(steering_angle) < to_radians(10.0)) {
            return max_velocity_;
        } else if (abs(steering_angle) >= to_radians(10.0) && abs(steering_angle) <= to_radians(20.0)) {
            return mid_velocity_;
        } else {
            return min_velocity_;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PursuitNode>());
    rclcpp::shutdown();
    return 0;
}