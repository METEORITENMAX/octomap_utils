#ifndef OCTOMAP_CONVERSIONS_HPP
#define OCTOMAP_CONVERSIONS_HPP


// Use ros2 timestamps
#include <rclcpp/rclcpp.hpp>
// Output of path in ROS2 format
#include <nav_msgs/msg/path.hpp>
#include <octomap/OcTree.h>
#include "octomap_calculations.hpp"

namespace octomap_utils {

nav_msgs::msg::Path create_pathMsg_from_points3d(const std::vector<octomap::point3d>& points3d) {

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = rclcpp::Clock().now();
    path_msg.header.frame_id = "map";

    for (size_t i = 0; i < points3d.size(); ++i) {

        auto point = points3d[i];
        tf2::Quaternion orientation;
        if (i < points3d.size() - 1) {
            orientation = calculateOrientation(points3d[i], points3d[i + 1]);
        } else {
            // Let the last point have the orientation of the second to last point
            orientation = calculateOrientation(points3d[i - 1], points3d[i]);
        }

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = rclcpp::Clock().now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = point.x();
        pose_stamped.pose.position.y = point.y();
        pose_stamped.pose.position.z = point.z();
        pose_stamped.pose.orientation.x = orientation.x();
        pose_stamped.pose.orientation.y = orientation.y();
        pose_stamped.pose.orientation.z = orientation.z();
        pose_stamped.pose.orientation.w = orientation.w();

        path_msg.poses.push_back(pose_stamped);
    }

    return path_msg;

}

}

#endif