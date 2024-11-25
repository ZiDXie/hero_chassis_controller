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

#include "realtime_tools/realtime_publisher.h"

namespace hero_chassis_controller
{
class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

  // 关节和pid
  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
  control_toolbox::Pid pid_front_left_, pid_front_right_, pid_back_left_, pid_back_right_;

  // 动态参数
  void cb(hero_chassis_controller::pidConfig& config, uint32_t level);
  std::shared_ptr<dynamic_reconfigure::Server<hero_chassis_controller::pidConfig>> server;

  // 订阅速度
  ros::Subscriber cmd_sub;
  void cmdvel_cb(const geometry_msgs::Twist::ConstPtr& msg);

  // 车的目标速度和参数
  // 车的参数从urdf文件中读取
  double vx, vy, wz = 0.0;
  double wheel_base = 0.4;
  double wheel_track = 0.4;
  double wheel_radius = 0.07625;

  // 发布里程计
  ros::Publisher odom_pub;
  tf2_ros::TransformBroadcaster odom_broadcaster;
  // 机器人最初从 “odom” 坐标系的原点开始。
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  // 实际的速度
  double vx_real = 0.0;
  double vy_real = 0.0;
  double vth_real = 0.0;

  // 模式切换
  bool chassis_mode = false;
  tf::TransformListener tf_listener;
  // 源坐标系
  geometry_msgs::Vector3Stamped global;
  //目标坐标系
  geometry_msgs::Vector3Stamped chassis;


  };
}

#endif //HERO_CHASSIS_CONTROLLER_H
