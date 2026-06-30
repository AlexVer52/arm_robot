#include <memory>
#include <chrono>
#include <thread>
#include <cmath>
#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <string>

#include "arm_robot_interfaces/msg/detected_object.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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
            detection_subscriber_ = node_->create_subscription<arm_robot_interfaces::msg::DetectedObject>(
                "/arm_robot/detections",
                1,
                std::bind(&TaskManagerNode::detectionCallback, this, std::placeholders::_1)
            );

            joint_state_subscriber_ =  node_->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10,
                std::bind(&TaskManagerNode::jointCallback, this, std::placeholders::_1)
            );
        }

    private:
        void detectionCallback(const arm_robot_interfaces::msg::DetectedObject::SharedPtr msg)
        {
            if (state_ != TaskState::IDLE) {
                RCLCPP_WARN(node_->get_logger(), "Currently busy, ignoring new detection");
                return;
            }

            auto planning_frame = arm_interface_->getPlanningFrame();

            geometry_msgs::msg::PointStamped object_camera_frame;
            object_camera_frame.header = msg->header;
            object_camera_frame.point = msg->point;

            geometry_msgs::msg::PointStamped object_in_planning_frame;

            try {
              object_in_planning_frame = tf_buffer_->transform(
                object_camera_frame,
                planning_frame,
                tf2::durationFromSec(0.5));
            } catch (const tf2::TransformException & ex) {
              RCLCPP_ERROR(node_->get_logger(), "TF transform failed: %s", ex.what());
              state_ = TaskState::IDLE;
              return;
            }
            {
              // 
              std::lock_guard<std::mutex> lock(detection_mutex_);
              latest_detection_ = object_in_planning_frame;
              last_detection_time_ = node_->now();
            }
          
            if (state_ != TaskState::IDLE) {
              return;  // Update tracking, but do not start another pick.
            }

            state_ = TaskState::PICKING;

            auto color = msg->color;
            RCLCPP_INFO(node_->get_logger(), "Detected color: %s", color.c_str());
            
            
            std::thread(
              &TaskManagerNode::executePickDropTask,
              this,
              object_in_planning_frame,
              color
            ).detach();
        }

        void executePickDropTask(geometry_msgs::msg::PointStamped object_camera_frame, std::string color)
        {
          RCLCPP_INFO(
            node_->get_logger(),
            "Detected object in planning frame at: x: %f, y: %f, z: %f, color: %s",
            object_camera_frame.point.x,
            object_camera_frame.point.y,
            object_camera_frame.point.z,
            color.c_str()
          ); 
          bool open_gripper = openGripper();
          if (!open_gripper) {
            arm_interface_->setNamedTarget("home");
            arm_interface_->move();
            state_ = TaskState::IDLE;
            return;
          }
          
          // Set the target pose for the arm
          bool pre_grapsed = executeGrasp(
            object_camera_frame.point.x,
            object_camera_frame.point.y,
            object_camera_frame.point.z + 0.08
          );
          if (!pre_grapsed) {
            arm_interface_->setNamedTarget("home");
            arm_interface_->move();
            state_ = TaskState::IDLE;
            return;
          }
          
          bool picked = executeGrasp(
            object_camera_frame.point.x,
            object_camera_frame.point.y,
            object_camera_frame.point.z
          );
          if (!picked) {
            arm_interface_->setNamedTarget("home");
            arm_interface_->move();
            state_ = TaskState::IDLE;
            return;
          }
          
          bool close_gripper = closeGripperAndCheckObject(); 
          if (!close_gripper) {
            arm_interface_->setNamedTarget("home");
            arm_interface_->move();
            state_ = TaskState::IDLE;
            return;
          }
          
          RCLCPP_INFO(node_->get_logger(), "Gripper position updated: %f", gripper_position); 
          if (gripper_position > -0.008 && has_gripper_position == true)
          {
            state_ = TaskState::HOLDING;
            arm_interface_->setNamedTarget("home");
            arm_interface_->move();

            bool drop = false;
            if (color == "blue"){
              drop = executeGrasp(-0.34, 0.16, 0.05);
            }
            if (color == "green"){
              drop = executeGrasp(-0.16, -0.32, 0.05);
            }
            if (color == "yellow"){
              drop = executeGrasp(-0.34, -0.16, 0.05);
            }
            if (color == "purple"){
              drop = executeGrasp(-0.46, 0, 0.05);
            }
            if (color == "red"){
              drop = executeGrasp(-0.16, 0.32, 0.05); 
            }
              

            if (!drop){
              RCLCPP_INFO(node_->get_logger(), "Failed to move to drop position");
              // Should send back to object pose to put it back to the original position
              executeGrasp(object_camera_frame.point.x, object_camera_frame.point.y, object_camera_frame.point.z - 0.15);
              gripper_interface_->setNamedTarget("open");
              gripper_interface_->move();
              arm_interface_->setNamedTarget("home");
              arm_interface_->move();
              state_ = TaskState::IDLE;
              return;
            }
            
            gripper_interface_->setNamedTarget("open");
            gripper_interface_->move();
            arm_interface_->setNamedTarget("home");
            arm_interface_->move();
            state_ = TaskState::IDLE;
          }
          else
          {
            RCLCPP_INFO(node_->get_logger(), "Failed to catch the item");
            arm_interface_->setNamedTarget("home");
            if (arm_interface_->move()) {
              RCLCPP_INFO(node_->get_logger(), "Back to home position");  // Log success
              std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for 2 seconds
              state_ = TaskState::IDLE;
            }
          }
        }

        bool isObjectNearGripper(double threshold = 0.07)
        {
          geometry_msgs::msg::PointStamped object;

          {
            std::lock_guard<std::mutex> lock(detection_mutex_);
          
            if ((node_->now() - last_detection_time_).seconds() > 1.0) {
              return false;
            }
          
            object = latest_detection_;
          }
        
          const auto gripper =
            arm_interface_->getCurrentPose("end_effector_link");
        
          const double dx =
            object.point.x - gripper.pose.position.x;
          const double dy =
            object.point.y - gripper.pose.position.y;
          const double dz =
            object.point.z - gripper.pose.position.z;
        
          return std::sqrt(dx * dx + dy * dy + dz * dz) < threshold;
        }

        bool executeGrasp(double x, double y, double z, double yaw = 0.0)
        {
          (void)yaw;  // Will be used later on when we have cube orientation detection

          // Set the target position for the arm
          arm_interface_->setPositionTarget(x, y, z);

          // Set tolerances for goal position and orientation
          arm_interface_->setGoalPositionTolerance(0.02);
          arm_interface_->setGoalOrientationTolerance(0.02);
        
          MoveGroupInterface::Plan plan;
          bool success = static_cast<bool>(arm_interface_->plan(plan));
        
          if (!success) {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed");
            return false;
          }

          // Execute the planned trajectory for the arm
          const bool executed = static_cast<bool>(arm_interface_->execute(plan));
          
          return executed;
        }

        bool closeGripperAndCheckObject()
        {
          gripper_interface_->setNamedTarget("close");
        
          const bool reached_closed_target = static_cast<bool>(gripper_interface_->move());
        
          std::this_thread::sleep_for(std::chrono::milliseconds(250));
        
          if (!has_gripper_position) {
            RCLCPP_ERROR(node_->get_logger(), "No gripper joint state");
            return false;
          }
        
          RCLCPP_INFO(
            node_->get_logger(),
            "Close result=%d, measured position=%f",
            reached_closed_target,
            gripper_position
          );
        
          const bool object_between_fingers = gripper_position > -0.008 && gripper_position < 0.018;
        
          if (object_between_fingers) {
            RCLCPP_INFO(node_->get_logger(), "Object detected in gripper");
            return true;
          }
        
          if (reached_closed_target) {
            RCLCPP_WARN(
              node_->get_logger(),
              "Gripper fully closed: probably no object"
            );
          } else {
            RCLCPP_ERROR(
              node_->get_logger(),
              "Gripper failed without valid object obstruction"
            );
          }
        
          return false;
        }

        bool openGripper()
        {
          gripper_interface_->setNamedTarget("open");
          if (gripper_interface_->move()) {
            RCLCPP_INFO(node_->get_logger(), "Gripper opened successfully");  // Log success
            std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for 2 seconds
            return true;
          } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open the gripper");
            return false;
          }
        }

        void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) 
        {
          for(size_t i = 0; i < msg->name.size(); i++)
          {
            if(msg->name[i] == "gripper_left_joint")
            {
              gripper_position = msg->position[i];
              has_gripper_position = true;
            }
          }
        }
        
        // Store them as member variables for later use in the callback
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<MoveGroupInterface> arm_interface_;
        std::shared_ptr<MoveGroupInterface> gripper_interface_;

        rclcpp::Subscription<arm_robot_interfaces::msg::DetectedObject>::SharedPtr detection_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

        rclcpp_action::Server<Grasp>::SharedPtr grasp_action_server_;
        rclcpp::Service<MoveHomePosition>::SharedPtr move_home_service_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        double gripper_position = 0.0;
        bool has_gripper_position = false;

        enum class TaskState
        {
          IDLE, // Waiting for a task
          PICKING,
          HOLDING,
          RECOVERING
        };

        std::atomic<TaskState> state_{TaskState::IDLE};

        geometry_msgs::msg::PointStamped latest_detection_;
        rclcpp::Time last_detection_time_;
        std::mutex detection_mutex_;
};

int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        
        // Creation of the Node here in main because MoveGroupInterface requires a valid ROS2 node to be passed to its constructor
        // Create the ROS2 node
        auto const node = std::make_shared<rclcpp::Node>(
          "task_manager_node",
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );

        // Create the MoveIt MoveGroup Interface for the "arm" and "gripper" planning group
        auto arm_interface = std::make_shared<MoveGroupInterface>(node, "arm");
        auto gripper_interface = std::make_shared<MoveGroupInterface>(node, "gripper");

        arm_interface->setNamedTarget("home");
        arm_interface->move();

        auto task_manager_node = std::make_shared<TaskManagerNode>(node, arm_interface, gripper_interface);

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();

        rclcpp::shutdown();
        return 0;
    }