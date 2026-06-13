#include <memory>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using moveit::planning_interface::MoveGroupInterface;

class TaskManagerNode
{
    public:
        TaskManagerNode(
            const rclcpp::Node::SharedPtr & node,
            std::shared_ptr<MoveGroupInterface> arm,
            std::shared_ptr<MoveGroupInterface> gripper
        )
        : node_(node), 
          arm_interface_(arm),
          gripper_interface_(gripper)
        {
            // Initialize the TF buffer and listener
              tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
              tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Create subriber for target pose
            detection_subscriber_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
                "/arm_robot/detections",
                10,
                std::bind(&TaskManagerNode::detectionCallback, this, std::placeholders::_1)
            );
        }

    private:
        void detectionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
        {
            if (is_busy_) {
                RCLCPP_WARN(node_->get_logger(), "Currently busy, ignoring new detection");
                return;
            }
            is_busy_ = true;

            auto planning_frame = arm_interface_->getPlanningFrame();

            geometry_msgs::msg::PointStamped object_in_planning_frame;
            try {
              object_in_planning_frame = tf_buffer_->transform(
                *msg,
                planning_frame,
                tf2::durationFromSec(0.5));
            } catch (const tf2::TransformException & ex) {
              RCLCPP_ERROR(node_->get_logger(), "TF transform failed: %s", ex.what());
              is_busy_ = false;
              return;
            }

            // Set the "open" position for the gripper if needed
            gripper_interface_->setNamedTarget("open");
            if (gripper_interface_->move()) {
              RCLCPP_INFO(node_->get_logger(), "Gripper opened successfully");  // Log success
              std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for 2 seconds
            } else {
              RCLCPP_ERROR(node_->get_logger(), "Failed to open the gripper");
            }

            // Define the target pose for the robot arm using the detected point with modification of frame
            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = object_in_planning_frame.point.x; 
            target_pose.position.y =  -object_in_planning_frame.point.y;
            target_pose.position.z = object_in_planning_frame.point.z + 0.05; // Adjust the target pose to be above the detected point
            target_pose.orientation.w = 1.0;

            RCLCPP_INFO(
              node_->get_logger(),
              "Detected object in planning frame at: x: %f, y: %f, z: %f",
              object_in_planning_frame.point.x,
              object_in_planning_frame.point.y,
              object_in_planning_frame.point.z
            );

            RCLCPP_INFO(
              node_->get_logger(),
              "Target pose set to: x: %f, y: %f, z: %f in frame: %s",
              target_pose.position.x,
              target_pose.position.y,
              target_pose.position.z,
              planning_frame.c_str()
            );

            // Set the target pose for the arm
            arm_interface_->setPoseTarget(target_pose);
                    
            // Set tolerances for goal position and orientation
            arm_interface_->setGoalPositionTolerance(0.02);
            arm_interface_->setGoalOrientationTolerance(0.02);

            // Plan the motion for the arm to reach the target pose
            MoveGroupInterface::Plan plan;
            bool success = static_cast<bool>(arm_interface_->plan(plan));
          
            // If planning succeeds, execute the planned motion
            if (success) {
              arm_interface_->execute(plan);
              std::this_thread::sleep_for(std::chrono::seconds(2));
            } else {
              RCLCPP_ERROR(node_->get_logger(), "Planning failed for the arm!");  // Log an error if planning fails
            }

            // Set the "close" position for the gripper and move it
            gripper_interface_->setNamedTarget("close");
            if (gripper_interface_->move()) {
              RCLCPP_INFO(node_->get_logger(), "Gripper closed successfully");  // Log success
              std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for 2 seconds
            } else {
              RCLCPP_ERROR(node_->get_logger(), "Failed to close the gripper");
            }

            is_busy_ = false;
        }
        
        // Store them as member variables for later use in the callback
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<MoveGroupInterface> arm_interface_;
        std::shared_ptr<MoveGroupInterface> gripper_interface_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr detection_subscriber_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        bool is_busy_ = false;
};

int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        
        // Create the ROS2 node
        auto const node = std::make_shared<rclcpp::Node>(
          "task_manager_node",
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );

        // Create the MoveIt MoveGroup Interface for the "arm" and "gripper" planning group
        auto arm_interface = std::make_shared<MoveGroupInterface>(node, "arm");
        auto gripper_interface = std::make_shared<MoveGroupInterface>(node, "gripper");

        auto task_manager_node = std::make_shared<TaskManagerNode>(node, arm_interface, gripper_interface);

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();

        rclcpp::shutdown();
        return 0;
    }