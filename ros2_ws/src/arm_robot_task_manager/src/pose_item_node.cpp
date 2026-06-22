#include <memory>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

class PoseItemNode: public rclcpp::Node
{
public:
    PoseItemNode():Node("pose_item_node")
    {
        // Create publisher for item pose
        item_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/arm_robot/item_pose", 
            10);

        // Create subriber for Gazebo pose Information through Gazebo because the bridge has an issue, we lose the name of the object pose
        const bool subscribed = gazebo_node_.Subscribe(
            "/world/RLTraining/pose/info",
            &PoseItemNode::poseCallback,
            this);

        if (!subscribed)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to Gazebo pose topic");
        }
    }
    
private:
    void poseCallback(const gz::msgs::Pose_V & msg)
    {
        // Take the pose of 'item'
        for (int i = 0; i < msg.pose_size(); ++i) {
            const auto & pose = msg.pose(i);

            if (pose.name() != "item") {
              continue;
            }

            geometry_msgs::msg::PoseStamped output;
            output.header.stamp = this->now();
            output.header.frame_id = "world";

            output.pose.position.x = pose.position().x();
            output.pose.position.y = pose.position().y();
            output.pose.position.z = pose.position().z();

            output.pose.orientation.x = pose.orientation().x();
            output.pose.orientation.y = pose.orientation().y();
            output.pose.orientation.z = pose.orientation().z();
            output.pose.orientation.w = pose.orientation().w();

            item_pose_publisher_->publish(output);
            return;
        }
    }

    gz::transport::Node gazebo_node_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr item_pose_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseItemNode>());
  rclcpp::shutdown();
  return 0;
}