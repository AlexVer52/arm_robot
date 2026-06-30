#include <memory>
#include <atomic>
#include <chrono>
#include <csignal>
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
#include <sensor_msgs/msg/joint_state.hpp>

#include "arm_robot_interfaces/action/grasp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "arm_robot_interfaces/srv/move_home_position.hpp"

#include "arm_robot_interfaces/msg/detected_object.hpp"

using moveit::planning_interface::MoveGroupInterface;

std::atomic_bool shutdown_requested{false};

void handleSignal(int)
{
  shutdown_requested = true;
}

class TaskManagerNode
{
    public:
        // Initialize for Action server
        using Grasp = arm_robot_interfaces::action::Grasp;
        using GoalHandleGrasp = rclcpp_action::ServerGoalHandle<Grasp>;
        using MoveHomePosition = arm_robot_interfaces::srv::MoveHomePosition;

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

            grasp_action_server_ = rclcpp_action::create_server<Grasp>(
              node_,
              "/arm_robot/grasp",
              std::bind(&TaskManagerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
              std::bind(&TaskManagerNode::handleCancel, this, std::placeholders::_1),
              std::bind(&TaskManagerNode::handleAccepted, this, std::placeholders::_1)
            );

            move_home_service_ = node_->create_service<MoveHomePosition>(
              "/arm_robot/move_home_position", 
              std::bind(&TaskManagerNode::handleMoveHome, this, std::placeholders::_1, std::placeholders::_2)
            );
        }

    private:
        void detectionCallback(const arm_robot_interfaces::msg::DetectedObject::SharedPtr msg)
        {
            if (is_busy_) {
                RCLCPP_WARN(node_->get_logger(), "Currently busy, ignoring new detection");
                return;
            }
            is_busy_ = true;

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
              is_busy_ = false;
              return;
            }

	            RCLCPP_INFO(
	              node_->get_logger(),
	              "Detected object in planning frame at: x: %f, y: %f, z: %f",
	              object_in_planning_frame.point.x,
	              object_in_planning_frame.point.y,
	              object_in_planning_frame.point.z
	            );

            std::thread(
              &TaskManagerNode::executePickDropTask,
              this,
              object_in_planning_frame
            ).detach();
        }

        void executePickDropTask(geometry_msgs::msg::PointStamped object_in_planning_frame)
        {
            bool open_gripper = openGripper();

	            if (!open_gripper) {
	              goHome();
	              is_busy_ = false;
	              return;
	            }

            // Set the target pose for the arm
            bool pre_grapsed = executeGrasp(
              object_in_planning_frame.point.x,
              object_in_planning_frame.point.y,
              object_in_planning_frame.point.z + 0.08
            );

	            if (!pre_grapsed) {
	              goHome();
	              is_busy_ = false;
	              return;
	            }

            bool picked = executeGrasp(
              object_in_planning_frame.point.x,
              object_in_planning_frame.point.y,
              object_in_planning_frame.point.z - 0.02
            );

	            if (!picked) {
	              goHome();
	              is_busy_ = false;
	              return;
	            }

            bool close_gripper = closeGripperAndCheckObject();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

	            if (!close_gripper) {
	              goHome();
	              is_busy_ = false;
	              return;
	            }

            RCLCPP_INFO(node_->get_logger(), "Gripper position updated: %f", gripper_position);

	            if (gripper_position > -0.008 && has_gripper_position == true)
	            {
	              if (!goHome()) {
	                is_busy_ = false;
	                return;
	              }

	              bool drop = executeGrasp(-0.08, 0.25, 0.1);
	              
	              if (!drop){
	                RCLCPP_INFO(node_->get_logger(), "Failed to move to drop position");
	                // Should send back to object pose to put it back to the original position
	                executeGrasp(object_in_planning_frame.point.x, object_in_planning_frame.point.y, object_in_planning_frame.point.z - 0.15);
	                gripper_interface_->setNamedTarget("open");
	                gripper_interface_->move();
	                goHome();
	                is_busy_ = false;
	                return;
	              }

	              gripper_interface_->setNamedTarget("open");
	              gripper_interface_->move();
	              goHome();
	              is_busy_ = false;
	            }
	            else
	            {
	              RCLCPP_INFO(node_->get_logger(), "Failed to catch the item");
	              if (goHome()) {
	                RCLCPP_INFO(node_->get_logger(), "Back to home position");  // Log success
	                std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for 2 seconds
	                is_busy_ = false;
	              }
	            }
	        }

        bool goHome()
        {
          arm_interface_->setNamedTarget("home");
          const bool success = static_cast<bool>(arm_interface_->move());

          if (!success) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to move arm home");
          }

          return success;
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
          std::this_thread::sleep_for(std::chrono::seconds(2));
          
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
        
        // Action server realted functions
        // Callback for when a new goal is received
        rclcpp_action::GoalResponse handleGoal(
          const rclcpp_action::GoalUUID & uuid,
          std::shared_ptr<const Grasp::Goal> goal)
        {
          RCLCPP_INFO(node_->get_logger(), "Received goal request");
          (void)uuid;
          (void)goal;

          if (is_busy_) {
            return rclcpp_action::GoalResponse::REJECT;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

        rclcpp_action::CancelResponse handleCancel(
          const std::shared_ptr<GoalHandleGrasp> goal_handle)
        {
          RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
          (void)goal_handle;
          return rclcpp_action::CancelResponse::ACCEPT;
        };
      
        void handleAccepted(
          const std::shared_ptr<GoalHandleGrasp> goal_handle)
        {
          // this needs to return quickly to avoid blocking the executor,
          // so we declare a lambda function to be called inside a new thread
          std::thread{std::bind(&TaskManagerNode::executeGraspAction, this, goal_handle)}.detach();
        };

        // Called when a new goal is accepted by the action server
        void executeGraspAction(const std::shared_ptr<GoalHandleGrasp> goal_handle)
        {
          RCLCPP_INFO(node_->get_logger(), "Executing goal");

          const auto goal = goal_handle->get_goal();

          auto feedback = std::make_shared<Grasp::Feedback>();
          auto result = std::make_shared<Grasp::Result>();

          const bool grasp_motion = executeGrasp(
            goal->target_pose.pose.position.x,
            goal->target_pose.pose.position.y, 
            goal->target_pose.pose.position.z
          );

          bool home_success = false;

          if (grasp_motion)
          {
            arm_interface_->setNamedTarget("home");
            home_success = static_cast<bool>(arm_interface_->move());
          }

          const bool gripper_indicates_object = has_gripper_position && gripper_position > -0.008;
          
          const bool success = grasp_motion && home_success && gripper_indicates_object;
          
          result->success = success;
          result->message = success ? "Grasp succeeded" : "Grasp failed";
          result->gripper_position = gripper_position;
        
          goal_handle->succeed(result);
        }

                
        // Service related function
	        void handleMoveHome(
	          const std::shared_ptr<MoveHomePosition::Request> request,
	          std::shared_ptr<MoveHomePosition::Response> response)
	        {
          (void)request;
          
          if (is_busy_)
          {
            response->success = false;
            response->message = "Task manager is busy";
            return;
          }

	          is_busy_ = true;

	          const bool success = goHome();
	          
	          is_busy_ = false;

          response->success = success;
          response->message = success ? "Arm reached home position" : "Failed to reach home position";
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
        bool is_busy_ = false;
        bool has_gripper_position = false;
};

int main(int argc, char * argv[])
	    {
	        rclcpp::init(
	          argc,
	          argv,
	          rclcpp::InitOptions(),
	          rclcpp::SignalHandlerOptions::None);

	        std::signal(SIGINT, handleSignal);
	        std::signal(SIGTERM, handleSignal);
	        
	        // Creation of the Node here in main because MoveGroupInterface requires a valid ROS2 node to be passed to its constructor
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

	        std::thread signal_watcher([&executor]() {
	          while (rclcpp::ok() && !shutdown_requested) {
	            std::this_thread::sleep_for(std::chrono::milliseconds(100));
	          }

	          if (shutdown_requested) {
	            executor.cancel();
	          }
	        });

	        executor.spin();

	        if (shutdown_requested && rclcpp::ok()) {
	          RCLCPP_INFO(node->get_logger(), "Shutdown requested, moving arm home");
	          arm_interface->setNamedTarget("home");
	          const bool home_success = static_cast<bool>(arm_interface->move());

	          if (!home_success) {
	            RCLCPP_ERROR(node->get_logger(), "Failed to move arm home during shutdown");
	          }
	        }

	        shutdown_requested = true;
	        if (signal_watcher.joinable()) {
	          signal_watcher.join();
	        }

	        rclcpp::shutdown();
	        return 0;
    }
