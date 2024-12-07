//
// Created by xie on 24-11-21.
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include "pluginlib/class_list_macros.h"

namespace hero_chassis_controller
{
// Square function
double HeroChassisController::square(double x)
{
  return x * x;
}

// Callback function implementation
void HeroChassisController::cb(hero_chassis_controller::pidConfig& config, uint32_t level)
{
  pid_front_left_.setGains(config.left_front_p, config.left_front_i, config.left_front_d, 0, 0);
  pid_front_right_.setGains(config.right_front_p, config.right_front_i, config.right_front_d, 0, 0);
  pid_back_left_.setGains(config.left_back_p, config.left_back_i, config.left_back_d, 0, 0);
  pid_back_right_.setGains(config.right_back_p, config.right_back_i, config.right_back_d, 0, 0);
  ROS_INFO("Update PID gains:");
  ROS_INFO("Front Left: P=%.2f, I=%.2f, D=%.2f", config.left_front_p, config.left_front_i, config.left_front_d);
  ROS_INFO("Front Right: P=%.2f, I=%.2f, D=%.2f", config.right_front_p, config.right_front_i, config.right_front_d);
  ROS_INFO("Back Left: P=%.2f, I=%.2f, D=%.2f", config.left_back_p, config.left_back_i, config.left_back_d);
  ROS_INFO("Back Right: P=%.2f, I=%.2f, D=%.2f", config.right_back_p, config.right_back_i, config.right_back_d);
}
void HeroChassisController::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
  last_time = ros::Time::now();
  if (!chassis_mode)
  {
    // Global information settings
    global.header.frame_id = "odom";
    global.header.stamp = ros::Time(0);
    global.vector.x = msg->linear.x;
    global.vector.y = msg->linear.y;
    global.vector.z = 0.0;

    // Convert to chassis coordinate system
    tf_listener.transformVector("base_link", global, chassis);
    vx = chassis.vector.x;
    vy = chassis.vector.y;
    wz = msg->angular.z;
  }
  else
  {
    vx = msg->linear.x;
    vy = msg->linear.y;
    wz = msg->angular.z;
  }
}

// Move joint function implementation
void HeroChassisController::move_joint(const ros::Time& time, const ros::Duration& period)
{
  // Expect speed
  double fl_exp = (vx - vy - (lx + ly) * wz) / wheel_radius;
  double fr_exp = (vx + vy + (lx + ly) * wz) / wheel_radius;
  double bl_exp = (vx + vy - (lx + ly) * wz) / wheel_radius;
  double br_exp = (vx - vy + (lx + ly) * wz) / wheel_radius;

  // Actual vel
  double fl_actual = front_left_joint_.getVelocity();
  double fr_actual = front_right_joint_.getVelocity();
  double bl_actual = back_left_joint_.getVelocity();
  double br_actual = back_right_joint_.getVelocity();

  // Calculate
  double fl_effort = pid_front_left_.computeCommand(fl_exp - fl_actual, period);
  double fr_effort = pid_front_right_.computeCommand(fr_exp - fr_actual, period);
  double bl_effort = pid_back_left_.computeCommand(bl_exp - bl_actual, period);
  double br_effort = pid_back_right_.computeCommand(br_exp - br_actual, period);

  // Power limit algorithm
  double a = effort_coeff * (square(fl_effort) + square(fr_effort) + square(bl_effort) + square(br_effort));
  double b = std::abs(fl_effort * fl_actual) + std::abs(fr_effort * fr_actual) + std::abs(bl_effort * bl_actual) +
             std::abs(br_effort * br_actual);
  double c = vel_coeff * (square(fl_actual) + square(fr_actual) + square(bl_actual) + square(br_actual)) - power_limit -
             power_offset;
  double zoom_coeff = (square(b) - 4 * a * c) > 0 ? ((-b + sqrt(square(b) - 4 * a * c)) / (2 * a)) : 0.;
  if (zoom_coeff > 1) {}
  else
  {
    fl_effort = fl_effort * zoom_coeff;
    fr_effort = fr_effort * zoom_coeff;
    bl_effort = bl_effort * zoom_coeff;
    br_effort = br_effort * zoom_coeff;
  }

  // Output
  front_left_joint_.setCommand(fl_effort);
  front_right_joint_.setCommand(fr_effort);
  back_left_joint_.setCommand(bl_effort);
  back_right_joint_.setCommand(br_effort);
}

// Update odom function implementation
void HeroChassisController::update_odom(const ros::Time& time, const ros::Duration& period)
{
  // Actual vel
  double fl_actual = front_left_joint_.getVelocity();
  double fr_actual = front_right_joint_.getVelocity();
  double bl_actual = back_left_joint_.getVelocity();
  double br_actual = back_right_joint_.getVelocity();

  // The real_speed is the base_link speed
  vx_real = (fl_actual + fr_actual + bl_actual + br_actual) * wheel_radius / 4;
  vy_real = (-fl_actual + fr_actual + bl_actual - br_actual) * wheel_radius / 4;
  vth_real = (-fl_actual + fr_actual - bl_actual + br_actual) * wheel_radius / (4 * (lx + ly));
  // Calculate odometer and convert to odom
  double dt = period.toSec();
  double dx = (vx_real * cos(th) - vy_real * sin(th)) * dt;
  double dy = (vx_real * sin(th) + vy_real * cos(th)) * dt;
  double dth = vth_real * dt;

  // Overlay
  x += dx;
  y += dy;
  th += dth;

  // Create a quaternion from yaw
  tf2::Quaternion q;
  q.setRPY(0, 0, th);
  geometry_msgs::Quaternion odom_quat = toMsg(q);

  // Publish tf transform
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  odom_broadcaster.sendTransform(odom_trans);

  // Publish odom
  nav_msgs::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = "odom";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx_real;
  odom.twist.twist.linear.y = vy_real;
  odom.twist.twist.angular.z = vth_real;

  odom_pub.publish(odom);
}

// power pub
void HeroChassisController::power_msg_pub()
{
  // Actual vel
  double fl_actual = front_left_joint_.getVelocity();
  double fr_actual = front_right_joint_.getVelocity();
  double bl_actual = back_left_joint_.getVelocity();
  double br_actual = back_right_joint_.getVelocity();

  // Calculate
  double fl_effort = front_left_joint_.getCommand();
  double fr_effort = front_right_joint_.getCommand();
  double bl_effort = back_left_joint_.getCommand();
  double br_effort = back_right_joint_.getCommand();

  std_msgs::Float64 power_limit_msg;
  power_limit_msg.data = power_limit;
  power_limit_pub.publish(power_limit_msg);
  std_msgs::Float64 power_msg;
  power_msg.data = std::abs(fl_effort * fl_actual) + std::abs(fr_effort * fr_actual) + std::abs(bl_effort * bl_actual) +
                   std::abs(br_effort * br_actual) +
                   effort_coeff * (square(fl_effort) + square(fr_effort) + square(bl_effort) + square(br_effort)) +
                   vel_coeff * (square(fl_actual) + square(fr_actual) + square(bl_actual) + square(br_actual));
  power_pub.publish(power_msg);
}

// Initialization function implementation
bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // Joint initialization
  front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

  // pid initialization
  pid_front_left_.initPid(0, 0.0, 0.0, 0, -0);
  pid_front_right_.initPid(0, 0.0, 0.0, 0, 0);
  pid_back_left_.initPid(0, 0.0, 0.0, 0, 0);
  pid_back_right_.initPid(0, 0.0, 0.0, 0, 0);

  // Read parameters from configuration file
  if (!controller_nh.getParam("/controller/chassis_mode", chassis_mode) ||
      !controller_nh.getParam("/controller/wheel_base", wheel_base) ||
      !controller_nh.getParam("/controller/wheel_track", wheel_track) ||
      !controller_nh.getParam("/controller/power_limit", power_limit) ||
      !controller_nh.getParam("/controller/power/effort_coeff", effort_coeff) ||
      !controller_nh.getParam("/controller/power/vel_coeff", vel_coeff) ||
      !controller_nh.getParam("/controller/power/power_offset", power_offset) ||
      !controller_nh.getParam("/controller/accel/linear/x", accel_x) ||
      !controller_nh.getParam("/controller/accel/linear/y", accel_y) ||
      !controller_nh.getParam("/controller/accel/angular/z", accel_wz))
  {
    ROS_ERROR("Failed to get param");
    return false;
  }

  // Dynamic parameters
  server = std::make_shared<dynamic_reconfigure::Server<hero_chassis_controller::pidConfig>>(controller_nh);
  server->setCallback(
      [this](auto&& PH1, auto&& PH2) { cb(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2)); });

  // cmd_sub subscription speed
  cmd_sub = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &HeroChassisController::cmd_vel_cb, this);

  // Publish
  odom_pub = root_nh.advertise<nav_msgs::Odometry>("odom", 10);
  power_limit_pub = root_nh.advertise<std_msgs::Float64>("power_limit", 10);
  power_pub = root_nh.advertise<std_msgs::Float64>("power", 10);

  lx = wheel_base / 2;
  ly = wheel_track / 2;

  return true;
}

// Status update function implementation
void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  if ((time - last_time).toSec() > timeout)
  {
    vx = 0;
    vy = 0;
    wz = 0;
  }

  move_joint(time, period);

  update_odom(time, period);

  power_msg_pub();
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}  // namespace hero_chassis_controller