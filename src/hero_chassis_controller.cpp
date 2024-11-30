//
// Created by xie on 24-11-21.
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include "pluginlib/class_list_macros.h"

namespace hero_chassis_controller
{

// Callback function implementation
void HeroChassisController::cb(hero_chassis_controller::pidConfig& config, uint32_t level)
{
  pid_front_left_.setGains(config.left_front_p, config.left_front_i, config.left_front_d, 1, -1);
  pid_front_right_.setGains(config.right_front_p, config.right_front_i, config.right_front_d, 1, -1);
  pid_back_left_.setGains(config.left_back_p, config.left_back_i, config.left_back_d, 1, -1);
  pid_back_right_.setGains(config.right_back_p, config.right_back_i, config.right_back_d, 1, -1);

  chassis_mode = config.chassis_mode;

  ROS_INFO("Update PID gains:");
  ROS_INFO("Front Left: P=%.2f, I=%.2f, D=%.2f", config.left_front_p, config.left_front_i, config.left_front_d);
  ROS_INFO("Front Right: P=%.2f, I=%.2f, D=%.2f", config.right_front_p, config.right_front_i, config.right_front_d);
  ROS_INFO("Back Left: P=%.2f, I=%.2f, D=%.2f", config.left_back_p, config.left_back_i, config.left_back_d);
  ROS_INFO("Back Right: P=%.2f, I=%.2f, D=%.2f", config.right_back_p, config.right_back_i, config.right_back_d);
  ROS_INFO("chassis_mode=%d", config.chassis_mode);
}

void HeroChassisController::cmdvel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
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
  pid_front_left_.initPid(0, 0.0, 0.0, 1, -1);
  pid_front_right_.initPid(0, 0.0, 0.0, 1, -1);
  pid_back_left_.initPid(0, 0.0, 0.0, 1, -1);
  pid_back_right_.initPid(0, 0.0, 0.0, 1, -1);

  // Read parameters from configuration file
  controller_nh.param("chassis_mode", chassis_mode, true);
  controller_nh.param("wheel_base", wheel_base, 0.4);
  controller_nh.param("wheel_track", wheel_track, 0.4);
  controller_nh.param("power/power_limit", power_limit, 100.0);
  controller_nh.param("power/effort_coeff", effort_coeff, 10.0);
  controller_nh.param("power/vel_coeff", vel_coeff, 0.0060);

  // Dynamic parameters
  server = std::make_shared<dynamic_reconfigure::Server<hero_chassis_controller::pidConfig>>(controller_nh);
  server->setCallback(
      [this](auto&& PH1, auto&& PH2) { cb(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2)); });

  // cmd_sub subscription speed
  cmd_sub = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &HeroChassisController::cmdvel_cb, this);

  // Publish odom
  odom_pub = root_nh.advertise<nav_msgs::Odometry>("odom", 10);

  return true;
}

// Status update function implementation
void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  // 期望速度,根据论文中的公式计算
  double lx = wheel_base / 2;
  double ly = wheel_track / 2;
  double fl_exp = (vx - vy - (lx + ly) * wz) / wheel_radius;
  double fr_exp = (vx + vy + (lx + ly) * wz) / wheel_radius;
  double bl_exp = (vx + vy - (lx + ly) * wz) / wheel_radius;
  double br_exp = (vx - vy + (lx + ly) * wz) / wheel_radius;

  // Actual speed
  double fl_actual = front_left_joint_.getVelocity();
  double fr_actual = front_right_joint_.getVelocity();
  double bl_actual = back_left_joint_.getVelocity();
  double br_actual = back_right_joint_.getVelocity();

  // calculate
  double fl_effort = pid_front_left_.computeCommand(fl_exp - fl_actual, period);
  double fr_effort = pid_front_right_.computeCommand(fr_exp - fr_actual, period);
  double bl_effort = pid_back_left_.computeCommand(bl_exp - bl_actual, period);
  double br_effort = pid_back_right_.computeCommand(br_exp - br_actual, period);

  // Power limit algorithm
  double P_out = (abs(fl_effort * fl_actual) + abs(fr_effort * fr_actual) + abs(bl_effort * bl_actual) +
                  abs(br_effort * br_actual));
  double P_in = P_out + effort_coeff * (pow(fl_effort, 2) + pow(fr_effort, 2) + pow(bl_effort, 2) + pow(br_effort, 2)) +
                vel_coeff * (pow(fl_actual, 2) + pow(fr_actual, 2) + pow(bl_actual, 2) + pow(br_actual, 2));
  if (P_in > power_limit)
  {
    double k =
        ((-P_out) +
         sqrt(pow(fl_effort * fl_actual, 2) + pow(fr_effort * fr_actual, 2) + pow(bl_effort * bl_actual, 2) +
              pow(br_effort * br_actual, 2) -
              4 * effort_coeff * (pow(fl_effort, 2) + pow(fr_effort, 2) + pow(bl_effort, 2) + pow(br_effort, 2)) *
                  (vel_coeff * (pow(fl_actual, 2) + pow(fr_actual, 2) + pow(bl_actual, 2) + pow(br_actual, 2)) -
                   power_limit))) /
        2 * effort_coeff * (pow(fl_effort, 2) + pow(fr_effort, 2) + pow(bl_effort, 2) + pow(br_effort, 2));
    fl_effort = fl_effort * k;
    fr_effort = fr_effort * k;
    bl_effort = bl_effort * k;
    br_effort = br_effort * k;
  }

  // Output
  front_left_joint_.setCommand(fl_effort);
  front_right_joint_.setCommand(fr_effort);
  back_left_joint_.setCommand(bl_effort);
  back_right_joint_.setCommand(br_effort);

  // odom
  double dt = period.toSec();

  // 计算实际速度,根据论文中的公式计算
  // The real_speed is the base_link speed
  vx_real = (fl_actual + fr_actual + bl_actual + br_actual) * wheel_radius / 4;
  vy_real = (-fl_actual + fr_actual + bl_actual - br_actual) * wheel_radius / 4;
  vth_real = (-fl_actual + fr_actual - bl_actual + br_actual) * wheel_radius / (4 * (lx + ly));

  // Calculate odometer and convert to odom
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

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}


