/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Template Controller for nav2py
 */

 #ifndef NAV2PY_TEMPLATE_CONTROLLER__TEMPLATE_CONTROLLER_HPP_
 #define NAV2PY_TEMPLATE_CONTROLLER__TEMPLATE_CONTROLLER_HPP_
 
 #include <string>
 #include <vector>
 #include <memory>
 
 #include "nav2py/controller.hpp"
 #include "rclcpp/rclcpp.hpp"
 #include "pluginlib/class_loader.hpp"
 #include "pluginlib/class_list_macros.hpp"
 
 namespace nav2py_template_controller
 {
 
 class TemplateController : public nav2py::Controller
 {
 public:
   TemplateController() = default;
   ~TemplateController() override = default;
 
   void configure(
     const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
     std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
     const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
 
   void cleanup() override;
   void activate() override;
   void deactivate() override;
   void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
 
   geometry_msgs::msg::TwistStamped computeVelocityCommands(
     const geometry_msgs::msg::PoseStamped & pose,
     const geometry_msgs::msg::Twist & velocity,
     nav2_core::GoalChecker * goal_checker) override;
 
   void setPlan(const nav_msgs::msg::Path & path) override;
 
 protected:
   void sendData(
     const geometry_msgs::msg::PoseStamped & pose,
     const geometry_msgs::msg::Twist & velocity = geometry_msgs::msg::Twist());
 
   rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
   std::shared_ptr<tf2_ros::Buffer> tf_;
   std::string plugin_name_;
   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
   rclcpp::Logger logger_ {rclcpp::get_logger("TemplateController")};
   rclcpp::Clock::SharedPtr clock_;
 
   rclcpp::Duration transform_tolerance_ {0, 0};
 
   nav_msgs::msg::Path global_plan_;
   std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;
 };
 
 }  // namespace nav2py_template_controller
 
 #endif  // NAV2PY_TEMPLATE_CONTROLLER__TEMPLATE_CONTROLLER_HPP_