#include "rclcpp/rclcpp.hpp"
#include "octomap/octomap.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap_utils/octomap_modifcations.hpp"
#include <visualization_msgs/msg/marker.hpp>



class OctoMapHeightModNode : public rclcpp::Node
{

private:
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr _sub_octomap_init_;
    std::string _topicIn_octomap_init = "octomap_heightmod/in/octomap_init";

    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr _pub_octomap_heightMod_;
    std::string _topicOut_octomap_heightMod = "octomap_heightmod/out/octomap_heightMod";



public:
    OctoMapHeightModNode()
    : Node("octomap_heightmod")
    {
        _sub_octomap_init_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            _topicIn_octomap_init, 10,
            std::bind(&OctoMapHeightModNode::octomap_callback, this, std::placeholders::_1));

        _pub_octomap_heightMod_ = this->create_publisher<octomap_msgs::msg::Octomap>(_topicOut_octomap_heightMod, 10);
    }



private:
    void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        // Konvertiere ROS-Nachricht in Octomap
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        if (!tree)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to deserialize Octomap message.");
            return;
        }

        // Stelle sicher, dass die Octomap ein OcTree ist
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
        if (!octree)
        {
            RCLCPP_ERROR(this->get_logger(), "Octomap is not an OcTree.");
            delete tree;
            return;
        }

        // Erstelle einen neuen OcTree
        auto octree_occupied = std::make_shared<octomap::OcTree>(octree->getResolution());

        // Iteriere durch alle Voxel und speichere die Occumpied Pixel in einer neuen map
        for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it)
        {

            if (octree->isNodeOccupied(*it))
            {
                octree_occupied->updateNode(it.getKey(), it->getLogOdds());

                // Prüfe die Voxel direkt über dem aktuellen Voxel
                checkAndAddFreeVoxelsAbove(octree_occupied.get(), it.getKey());
            }

        }

        // TODO: Add a parameter if it should be inflated
        octomap_utils::inflateFreeSpaces(octree_occupied); 


        // Konvertiere die Octomap zurück in eine ROS-Nachricht
        octomap_msgs::msg::Octomap out_msg;
        octomap_msgs::fullMapToMsg(*octree_occupied, out_msg);
        out_msg.header = msg->header; // Behalte den ursprünglichen Header

        // Veröffentliche die modifizierte Octomap
        _pub_octomap_heightMod_->publish(out_msg);

        // Speicher freigeben
        delete tree;
    }


    void checkAndAddFreeVoxelsAbove(octomap::OcTree* octree, const octomap::OcTreeKey& key)
    {
        octomap::OcTreeKey key_above1 = key;
        key_above1[2] += 1; // Ein Voxel über dem aktuellen Voxel
        octomap::OcTreeKey key_above2 = key;
        key_above2[2] += 2; // Zwei Voxel über dem aktuellen Voxel
        octomap::OcTreeKey key_above3 = key;
        key_above3[2] += 3; // Zwei Voxel über dem aktuellen Voxel



        octomap::OcTreeNode* node_above1 = octree->search(key_above1);
        octomap::OcTreeNode* node_above2 = octree->search(key_above2);
        octomap::OcTreeNode* node_above3 = octree->search(key_above3);

        bool is_free1 = node_above1 && !octree->isNodeOccupied(node_above1);
        bool is_free2 = node_above2 && !octree->isNodeOccupied(node_above2);
        bool is_free3 = node_above3 && !octree->isNodeOccupied(node_above3);


        if (!is_free1)
        {
            octree->updateNode(key_above1, false); // Setze als 'free'
        }

        if (!is_free2)
        {
            octree->updateNode(key_above2, false); // Setze als 'free'
        }

        if (!is_free3)
        {
            octree->updateNode(key_above3, false); // Setze als 'free'
        }

    }

};




int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OctoMapHeightModNode>());
    rclcpp::shutdown();
    return 0;
}