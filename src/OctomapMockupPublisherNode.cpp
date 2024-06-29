#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include "octomap_utils/octomap_mockups.hpp"

class OctomapMockupPublisherNode : public rclcpp::Node
{

private:

    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr _pub_octoMapMockup_;
    std::string _topicOut_octoMapMockup = "octomap_mockup_publisher/out/octoMapMockup";
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<octomap::OcTree> octree_;

public:
    OctomapMockupPublisherNode()
    : Node("octomap_mockup_publisher")
    {
        // Create a mockup OcTree
        octree_ = octomap_utils::create_ocTreeMockup();
        
        // Create a publisher for the octomap message
        _pub_octoMapMockup_ = this->create_publisher<octomap_msgs::msg::Octomap>(this->_topicOut_octoMapMockup, 10);

        // Create a WallTimer to publish the message periodically
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&OctomapMockupPublisherNode::publish_octomap, this)
        );
    }

private:
    void publish_octomap()
    {
        auto msg = std::make_shared<octomap_msgs::msg::Octomap>();
        if (octomap_msgs::fullMapToMsg(*octree_, *msg)) {
            msg->header.frame_id = "map";
            msg->header.stamp = this->get_clock()->now();
            _pub_octoMapMockup_->publish(*msg);
            RCLCPP_INFO(this->get_logger(), "Published OcTree");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert OcTree to message");
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctomapMockupPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
