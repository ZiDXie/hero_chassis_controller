//
// Created by xie on 24-11-21.
//

#ifndef HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_H

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <ros/ros.h>

namespace hero_chassis_controller
{
  class HeroChassisController: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
  public:
    HeroChassisController() = default;
    ~HeroChassisController() override = default;

    bool init(hardware_interface::VelocityJointInterface* effort_joint_interface, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void starting(const ros::Time& time) override;
    void stopping(const ros::Time& time) override;

    hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
    control_toolbox::Pid pid_front_left_, pid_front_right_, pid_back_left_, pid_back_right_;

  };
}

#endif //HERO_CHASSIS_CONTROLLER_H
