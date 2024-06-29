#ifndef OCTOMAP_CALCULATIONS_HPP
#define OCTOMAP_CALCULATIONS_HPP

#include <octomap/OcTree.h>
// tf2 Helper to calculate orientation from point list
#include <tf2/LinearMath/Quaternion.h>

namespace octomap_utils {

tf2::Quaternion calculateOrientation(const octomap::point3d& point1, const octomap::point3d& point2) {
    double dx = point2.x() - point1.x();
    double dy = point2.y() - point1.y();
    double dz = point2.z() - point1.z();

    tf2::Vector3 axis(dx, dy, dz);
    axis.normalize();

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, std::atan2(dy, dx)); // Calculate yaw based on dx and dy
    return quaternion;
}

}

#endif