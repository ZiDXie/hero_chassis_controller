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
void HeroChassisController::cmdcb(const geometry_msgs::Twist::ConstPtr& msg)
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
  controller_nh.getParam("wheel_base", wheel_base);
  controller_nh.getParam("wheel_track", wheel_track);
  controller_nh.getParam("wheel_radius", wheel_radius);
  // cmd_sub订阅速度
  cmd_sub = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &HeroChassisController::cmdcb, this);

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

  // 输出
  front_left_joint_.setCommand(fl_effort);
  front_right_joint_.setCommand(fr_effort);
  back_left_joint_.setCommand(bl_effort);
  back_right_joint_.setCommand(br_effort);
    }

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}


