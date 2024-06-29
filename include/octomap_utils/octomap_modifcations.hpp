#ifndef OCTOMAP_MODDIFICATIONS_HPP
#define OCTOMAP_MODDIFICATIONS_HPP

#include <rclcpp/rclcpp.hpp>
#include <octomap/OcTree.h>



namespace octomap_utils {



// void inflateFreeSpaces(std::shared_ptr<octomap::OcTree> tree)
// {
//     std::vector<octomap::point3d> toUpdate;
//     for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
//         if (!tree->isNodeOccupied(*it)) {
//             // Inflate the free space by adding neighboring free voxels
//             octomap::point3d coord = it.getCoordinate();
//             toUpdate.push_back(coord + octomap::point3d(0.05, 0, 0));
//             toUpdate.push_back(coord + octomap::point3d(-0.05, 0, 0));
//             toUpdate.push_back(coord + octomap::point3d(0, 0.05, 0));
//             toUpdate.push_back(coord + octomap::point3d(0, -0.05, 0));
//             toUpdate.push_back(coord + octomap::point3d(0, 0, 0.05));
//             toUpdate.push_back(coord + octomap::point3d(0, 0, -0.05));
//         }
//     }

//     for (auto& coord : toUpdate) {
//         tree->updateNode(coord, false);
//     }
// }


void inflateFreeSpaces(std::shared_ptr<octomap::OcTree> tree)
{
    std::vector<octomap::point3d> toUpdate;
    const double resolution = tree->getResolution();  // The resolution of the OcTree
    const double inflation_radius = 0.1;  // The radius within which to inflate free voxels
    const double step_size = 0.02;  // The step size for inflation

    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
        if (!tree->isNodeOccupied(*it)) {
            octomap::point3d coord = it.getCoordinate();
            // Inflate around the free voxel within the specified radius
            for (double dx = -inflation_radius; dx <= inflation_radius; dx += step_size) {
                for (double dy = -inflation_radius; dy <= inflation_radius; dy += step_size) {
                    for (double dz = -inflation_radius; dz <= inflation_radius; dz += step_size) {
                        if (dx != 0.0 || dy != 0.0 || dz != 0.0) {
                            octomap::point3d neighbor = coord + octomap::point3d(dx, dy, dz);
                            toUpdate.push_back(neighbor);
                        }
                    }
                }
            }
        }
    }

    for (const auto& coord : toUpdate) {
        tree->updateNode(coord, false);  // Mark as free
    }
}




}



#endif