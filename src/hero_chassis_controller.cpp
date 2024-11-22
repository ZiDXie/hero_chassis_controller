//
// Created by xie on 24-11-21.
//


#include "hero_chassis_controller/hero_chassis_controller.h"
#include "pluginlib/class_list_macros.h"

namespace hero_chassis_controller
{
    bool HeroChassisController::init(hardware_interface::VelocityJointInterface* effort_joint_interface, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh){
      // 关节初始化
      front_left_joint_ = effort_joint_interface->getHandle("front_left_wheel_joint");
      front_right_joint_ = effort_joint_interface->getHandle("front_right_wheel_joint");
      back_left_joint_ = effort_joint_interface->getHandle("back_left_wheel_joint");
      back_right_joint_ = effort_joint_interface->getHandle("back_right_wheel_joint");

      //pid初始化
      pid_front_left_.init(ros::NodeHandle(controller_nh, "front_left_wheel_joint"));
      pid_front_right_.init(ros::NodeHandle(controller_nh, "front_right_wheel_joint"));
      pid_back_left_.init(ros::NodeHandle(controller_nh, "back_left_wheel_joint"));
      pid_back_right_.init(ros::NodeHandle(controller_nh, "back_right_wheel_joint"));


      return true;
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hero_chassis_controller");
    ros::NodeHandle nh;

    return 0;
}
