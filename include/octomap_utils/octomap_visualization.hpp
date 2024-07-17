#ifndef OCTOMAP_VISUALIZATION_HPP
#define OCTOMAP_VISUALIZATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <octomap/OcTree.h>



namespace octomap_utils {



visualization_msgs::msg::MarkerArray octomapToMarkerArray(const octomap::OcTree& octomap_tree) {

    visualization_msgs::msg::MarkerArray marker_array;

    size_t id = 0;
    for (octomap::OcTree::leaf_iterator it = octomap_tree.begin_leafs(), end = octomap_tree.end_leafs(); it != end; ++it) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; // Set the frame ID
        marker.header.stamp = rclcpp::Clock().now(); // Set the timestamp
        marker.id = id; // Unique ID for each marker
        marker.type = visualization_msgs::msg::Marker::CUBE; // Marker type (you may need to adjust this)
        marker.action = visualization_msgs::msg::Marker::ADD; // Action to take (ADD, DELETE, etc.)
        marker.pose.position.x = it.getX(); // Set marker position
        marker.pose.position.y = it.getY();
        marker.pose.position.z = it.getZ();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = octomap_tree.getResolution(); // Set marker scale
        marker.scale.y = octomap_tree.getResolution();
        marker.scale.z = octomap_tree.getResolution();

        // if (octomap_tree.isNodeOccupied(*it)) {
        double occupancy = it->getOccupancy();
        if (occupancy > octomap_tree.getOccupancyThres()) {

            marker.color.a = 0.5; // Set marker alpha (transparency)
            marker.color.r = 0.0; // Set marker color (red)
            marker.color.g = 1.0;
            marker.color.b = 0.0;

        } else {

            marker.color.a = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

        }

        marker_array.markers.push_back(marker); // Add marker to the marker array

        id++;
    }

    return marker_array;
}



visualization_msgs::msg::Marker point3dToMarker(const octomap::point3d& point, const std::string& frame_id, int marker_id, uint32_t markerType = visualization_msgs::msg::Marker::SPHERE) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id; // Set the frame ID
    marker.header.stamp = rclcpp::Clock().now(); // Set the timestamp
    marker.id = marker_id; // Unique ID for the marker
    marker.type = markerType; // Marker type (you may need to adjust this)
    marker.action = visualization_msgs::msg::Marker::ADD; // Action to take (ADD, DELETE, etc.)
    marker.pose.position.x = point.x(); // Set marker position
    marker.pose.position.y = point.y();
    marker.pose.position.z = point.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2; // Set marker scale
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0; // Set marker alpha (transparency)
    marker.color.r = 0.0; // Set marker color (red)
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    return marker;

}



visualization_msgs::msg::Marker visualizePath(const std::vector<octomap::point3d>& path) {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.header.stamp = rclcpp::Clock().now();
    path_marker.ns = "path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.scale.x = 0.05;
    path_marker.color.a = 1.0;
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0;
    path_marker.color.b = 0.0;

    for (const auto& point : path) {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        path_marker.points.push_back(p);
    }

    marker_array.markers.push_back(path_marker);

    return path_marker;
}



}



#endif