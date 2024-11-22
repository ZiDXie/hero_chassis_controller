//
// Created by xie on 24-11-21.
//

#ifndef HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_H

#include "control_toolbox/pid.h"
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include "dynamic_reconfigure/server.h"
#include "hero_chassis_controller/pidConfig.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

namespace hero_chassis_controller
{
  class HeroChassisController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
  {
  public:
    HeroChassisController() = default;
    ~HeroChassisController() override = default;

    bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    // void starting(const ros::Time& time) override;
    // void stopping(const ros::Time& time) override;

    // 关节和pid
    hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
    control_toolbox::Pid pid_front_left_, pid_front_right_, pid_back_left_, pid_back_right_;

    //动态参数
    void cb(hero_chassis_controller::pidConfig &config, uint32_t level);
    dynamic_reconfigure::Server<hero_chassis_controller::pidConfig> server;

    //订阅速度
    ros::Subscriber cmd_sub;
    void cmdcb(const geometry_msgs::Twist::ConstPtr& msg);

    //车的速度和参数
    double vx,vy,wz;
    double wheel_base;
    double track_width;

  };
}

#endif //HERO_CHASSIS_CONTROLLER_H
