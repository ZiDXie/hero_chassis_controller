//
// Created by xie on 24-11-21.
//

#ifndef HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_H

#include "control_toolbox/pid.h"
#include "hardware_interface/joint_command_interface.h"
#include "controller_interface/controller.h"
#include "dynamic_reconfigure/server.h"
#include "hero_chassis_controller/pidConfig.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "rm_msgs/ChassisCmd.h"

#include "realtime_tools/realtime_publisher.h"

namespace hero_chassis_controller
{
/*!
 * Main class for the node to handle the ROS interfacing.
 */
class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  /*!
   * Constructor.
   */
  HeroChassisController() = default;

  /*!
   * Destructor.
   */
  ~HeroChassisController() override = default;

private:
  /*!
   * init the ROS parameters.
   * @return true if successful.
   */
  bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  /*!
   * update the ROS status
   */
  void update(const ros::Time& time, const ros::Duration& period) override;

  /*!
   * ROS topic callback method.
   */
  void cmdvel_cb(const geometry_msgs::Twist::ConstPtr& msg);

  /*!
   * ROS service server callback.
   */
  void cb(hero_chassis_controller::pidConfig& config, uint32_t level);

  //! ROS service server.
  std::shared_ptr<dynamic_reconfigure::Server<pidConfig>> server;

  //! ROS topic subscriber.
  // Subscription speed
  ros::Subscriber cmd_sub;

  // Joints and PID
  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
  control_toolbox::Pid pid_front_left_, pid_front_right_, pid_back_left_, pid_back_right_;

  // Target speed and parameters of the car
  // The parameters of the car are in the URDF
  double vx, vy, wz = 0.0;
  double wheel_base;
  double wheel_track;
  double wheel_radius = 0.07625;

  // Publish the odom
  ros::Publisher odom_pub;
  tf2_ros::TransformBroadcaster odom_broadcaster;

  // The robot initially starts from the origin of the "odom" coordinate system.
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  // Actual speed
  double vx_real = 0.0;
  double vy_real = 0.0;
  double vth_real = 0.0;

  // Mode Switching
  bool chassis_mode;
  tf::TransformListener tf_listener;

  // Source coordinate system
  geometry_msgs::Vector3Stamped global;

  // Target coordinate system
  geometry_msgs::Vector3Stamped chassis;

  // Power limit
  double power_limit;
  double effort_coeff;
  double vel_coeff;
  double square(double x);
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_H
