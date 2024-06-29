#ifndef OCTOMAP_MOCKUPS_HPP
#define OCTOMAP_MOCKUPS_HPP



#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>



namespace octomap_utils {

// TODO: Add map-center as parameter
// NOTE: v02 was to test if the astar planer can't deal with negative height (it can't)
std::shared_ptr<octomap::OcTree> create_ocTreeMockup_v02(double resolution_octree = 0.1, size_t mapSize_xy = 100u, size_t mapHeight = 5u) {

    auto tree = std::make_shared<octomap::OcTree>(resolution_octree);

    // Größe des Würfels
    double map_size = (int) mapSize_xy; // von -1 bis 1 in x, y und z

    // insert some measurements of occupied cells as a floor mockup


    for (int x=-map_size; x<map_size; x++) {
        for (int y=-map_size; y<map_size; y++) {

            octomap::point3d endpoint ((float) x*0.02f-0.5f, (float) y*0.02f-0.5f, (float) -0.3);
            // NOTE: You need this shift if z goes negative
            endpoint.z() += 0.1;
            tree->updateNode(endpoint, true);  // integrate 'free' measurement

        }
    }

    for (int x=-map_size; x<map_size; x++) {
        for (int y=-map_size; y<map_size; y++) {

            octomap::point3d endpoint ((float) x*0.02f-0.5f, (float) y*0.02f-0.5f, (float) -0.2);
            // NOTE: You need this shift if z goes negative
            endpoint.z() += 0.1;
            tree->updateNode(endpoint, false);  // integrate 'free' measurement

        }
    }

    for (int x=-map_size; x<map_size; x++) {
        for (int y=-map_size; y<map_size; y++) {

            octomap::point3d endpoint ((float) x*0.02f-0.5f, (float) y*0.02f-0.5f, (float) -0.1);
            // NOTE: You need this shift if z goes negative
            endpoint.z() += 0.1;
            tree->updateNode(endpoint, false);  // integrate 'free' measurement

        }
    }

    // insert some measurements of free cells

    for (int x=-map_size; x<map_size; x++) {
        for (int y=-map_size; y<map_size; y++) {

            octomap::point3d endpoint ((float) x*0.02f-0.5f, (float) y*0.02f-0.5f, (float) 0.f);
            tree->updateNode(endpoint, false);  // integrate 'free' measurement

        }
    }

    for (int x=-map_size; x<map_size; x++) {
        for (int y=-map_size; y<map_size; y++) {

            octomap::point3d endpoint ((float) x*0.02f-0.5f, (float) y*0.02f-0.5f, (float) 0.1f);
            tree->updateNode(endpoint, false);  // integrate 'free' measurement

        }
    }

    for (int x=-map_size; x<map_size; x++) {
        for (int y=-map_size; y<map_size; y++) {

            octomap::point3d endpoint ((float) x*0.02f-0.5f, (float) y*0.02f-0.5f, (float) 0.2f);
            tree->updateNode(endpoint, false);  // integrate 'free' measurement

        }
    }

    for (int x=-map_size; x<map_size; x++) {
        for (int y=-map_size; y<map_size; y++) {

            octomap::point3d endpoint ((float) x*0.02f-0.5f, (float) y*0.02f-0.5f, (float) 0.3f);
            tree->updateNode(endpoint, false);  // integrate 'free' measurement

        }
    }

    for (int x=-map_size; x<map_size; x++) {
        for (int y=-map_size; y<map_size; y++) {

            octomap::point3d endpoint ((float) x*0.02f-0.5f, (float) y*0.02f-0.5f, (float) 0.4f);
            tree->updateNode(endpoint, false);  // integrate 'free' measurement

        }
    }
    // Optional: Stellen Sie sicher, dass alle Nodes auf der kleinsten Auflösungsebene sind
    tree->updateInnerOccupancy();

    return tree;

}



std::shared_ptr<octomap::OcTree> create_ocTreeMockup(double resolution_octree = 0.1, size_t mapSize_xy = 100u, size_t mapHeight = 5u) {

    auto tree = std::make_shared<octomap::OcTree>(resolution_octree);

    // Größe des Würfels
    double map_size = (int) mapSize_xy; // von -1 bis 1 in x, y und z

    // insert some measurements of occupied cells as a floor mockup

    for (int x=-map_size; x<map_size; x++) {
        for (int y=-map_size; y<map_size; y++) {

            octomap::point3d endpoint ((float) x*0.02f-0.5f, (float) y*0.02f-0.5f, (float) -0.1);
            // NOTE: You need this shift if z goes negative
            endpoint.z() += 0.1;
            tree->updateNode(endpoint, true);  // integrate 'free' measurement

        }
    }

    // insert some measurements of free cells

    for (int x=-map_size; x<map_size; x++) {
        for (int y=-map_size; y<map_size; y++) {

            octomap::point3d endpoint ((float) x*0.02f-0.5f, (float) y*0.02f-0.5f, (float) 0.f);
            tree->updateNode(endpoint, false);  // integrate 'free' measurement

        }
    }

    for (int x=-map_size; x<map_size; x++) {
        for (int y=-map_size; y<map_size; y++) {

            octomap::point3d endpoint ((float) x*0.02f-0.5f, (float) y*0.02f-0.5f, (float) 0.1f);
            tree->updateNode(endpoint, false);  // integrate 'free' measurement

        }
    }

    for (int x=-map_size; x<map_size; x++) {
        for (int y=-map_size; y<map_size; y++) {

            octomap::point3d endpoint ((float) x*0.02f-0.5f, (float) y*0.02f-0.5f, (float) 0.2f);
            tree->updateNode(endpoint, false);  // integrate 'free' measurement

        }
    }

    for (int x=-map_size; x<map_size; x++) {
        for (int y=-map_size; y<map_size; y++) {

            octomap::point3d endpoint ((float) x*0.02f-0.5f, (float) y*0.02f-0.5f, (float) 0.3f);
            tree->updateNode(endpoint, false);  // integrate 'free' measurement

        }
    }

    for (int x=-map_size; x<map_size; x++) {
        for (int y=-map_size; y<map_size; y++) {

            octomap::point3d endpoint ((float) x*0.02f-0.5f, (float) y*0.02f-0.5f, (float) 0.4f);
            tree->updateNode(endpoint, false);  // integrate 'free' measurement

        }
    }
    // Optional: Stellen Sie sicher, dass alle Nodes auf der kleinsten Auflösungsebene sind
    tree->updateInnerOccupancy();

    return tree;

}



}



#endif