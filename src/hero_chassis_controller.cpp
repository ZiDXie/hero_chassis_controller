//
// Created by xie on 24-11-21.
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include "pluginlib/class_list_macros.h"

namespace hero_chassis_controller
{

// 回调函数实现
void HeroChassisController::cb(hero_chassis_controller::pidConfig& config, uint32_t level)
{
  pid_front_left_.setGains(config.left_front_p, config.left_front_i, config.left_front_d, 100, 0);
  pid_front_right_.setGains(config.right_front_p, config.right_front_i, config.right_front_d, 100, 0);
  pid_back_left_.setGains(config.left_back_p, config.left_back_i, config.left_back_d, 100, 0);
  pid_back_right_.setGains(config.right_back_p, config.right_back_i, config.right_back_d, 100, 0);

  ROS_INFO("Update PID gains:");
  ROS_INFO("Front Left: P=%.2f, I=%.2f, D=%.2f", config.left_front_p, config.left_front_i, config.left_front_d);
  ROS_INFO("Front Right: P=%.2f, I=%.2f, D=%.2f", config.right_front_p, config.right_front_i, config.right_front_d);
  ROS_INFO("Back Left: P=%.2f, I=%.2f, D=%.2f", config.left_back_p, config.left_back_i, config.left_back_d);
  ROS_INFO("Back Right: P=%.2f, I=%.2f, D=%.2f", config.right_back_p, config.right_back_i, config.right_back_d);
}
void HeroChassisController::cmdvel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
  vx = msg->linear.x;
  vy = msg->linear.y;
  wz = msg->angular.z;
}

// 初始化函数实现
bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // 关节初始化
  front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

  // pid初始化
  pid_front_left_.initPid(0, 0.0, 0.0, 100, 0.0);
  pid_front_right_.initPid(0, 0.0, 0.0, 100, 0.0);
  pid_back_left_.initPid(0, 0.0, 0.0, 100, 0.0);
  pid_back_right_.initPid(0, 0.0, 0.0, 100, 0.0);

  // 动态参数
  server = std::make_shared<dynamic_reconfigure::Server<hero_chassis_controller::pidConfig>>(controller_nh);
  server->setCallback(boost::bind(&HeroChassisController::cb, this, _1, _2));
  // 在配置文件中读取参数
  // if (!controller_nh.getParam("wheel_base", wheel_base) || wheel_base <= 0)
  // {
  //   ROS_ERROR("Invalid or missing parameter: wheel_base");
  //   return false;
  // }
  // if (!controller_nh.getParam("wheel_track", wheel_track) || wheel_track <= 0)
  // {
  //   ROS_ERROR("Invalid or missing parameter: wheel_track");
  //   return false;
  // }
  // if (!controller_nh.getParam("wheel_radius", wheel_radius) || wheel_radius <= 0)
  // {
  //   ROS_ERROR("Invalid or missing parameter: wheel_radius");
  //   return false;
  // }

  // cmd_sub订阅速度
  cmd_sub = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &HeroChassisController::cmdvel_cb, this);

  // 发布里程计
  odom_pub = root_nh.advertise<nav_msgs::Odometry>("odom", 10);

  return true;
}

// 状态更新函数实现
void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  // 期望速度,根据论文中的公式计算
  double lx = wheel_base / 2;
  double ly = wheel_track / 2;
  double fl_exp = (vx - vy - (lx + ly) * wz) / wheel_radius;
  double fr_exp = (vx + vy + (lx + ly) * wz) / wheel_radius;
  double bl_exp = (vx + vy - (lx + ly) * wz) / wheel_radius;
  double br_exp = (vx - vy + (lx + ly) * wz) / wheel_radius;

  // 实际速度
  double fl_actual = front_left_joint_.getVelocity();
  double fr_actual = front_right_joint_.getVelocity();
  double bl_actual = back_left_joint_.getVelocity();
  double br_actual = back_right_joint_.getVelocity();

  // 计算
  double fl_effort = pid_front_left_.computeCommand(fl_exp - fl_actual, period);
  double fr_effort = pid_front_right_.computeCommand(fr_exp - fr_actual, period);
  double bl_effort = pid_back_left_.computeCommand(bl_exp - bl_actual, period);
  double br_effort = pid_back_right_.computeCommand(br_exp - br_actual, period);

  // 调试信息
  //  ROS_INFO_STREAM("Front Left: " << fl_effort << ", " << fl_exp << ", " << fl_actual);
  //  ROS_INFO_STREAM("Front Right: " << fr_effort << ", " << fr_exp << ", " << fr_actual);
  //  ROS_INFO_STREAM("Back Left: " << bl_effort << ", " << bl_exp << ", " << bl_actual);
  //  ROS_INFO_STREAM("Back Right: " << br_effort << ", " << br_exp << ", " << br_actual);

  // 输出
  front_left_joint_.setCommand(fl_effort);
  front_right_joint_.setCommand(fr_effort);
  back_left_joint_.setCommand(bl_effort);
  back_right_joint_.setCommand(br_effort);

  // odom里程计
  double dt = period.toSec();
  // 计算实际速度,根据论文中的公式计算
  // vx_real 是机器人在自身坐标系中的 x 方向速度。
  // vy_real 是机器人在自身坐标系中的 y 方向速度。
  vx_real = (fl_actual + fr_actual + bl_actual + br_actual) * wheel_radius / 4;
  vy_real = (-fl_actual + fr_actual + bl_actual - br_actual) * wheel_radius / 4;
  vth_real = (-fl_actual - fr_actual + bl_actual + br_actual) * wheel_radius / (4 * (lx + ly));
  // 在机器人速度的情况下，以典型方式计算里程计,换算成odom
  double dx = (vx_real * cos(th) - vy_real * sin(th)) * dt;
  double dy = (vx_real * sin(th) + vy_real * cos(th)) * dt;
  double dth = vth_real * dt;
  // 速度叠加
  x += dx;
  y += dy;
  th += dth;

  // 从 yaw 创建一个四元数
  tf2::Quaternion q;
  q.setRPY(0, 0, th);
  geometry_msgs::Quaternion odom_quat = tf2::toMsg(q);
  // 发布tf变换
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  odom_broadcaster.sendTransform(odom_trans);

  // 发布odom
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


