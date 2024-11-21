//
// Created by xie on 24-11-21.
//


#include "hero_chassis_controller/hero_chassis_controller.h"
#include "pluginlib/class_list_macros.h"

namespace hero_chassis_controller
{
    bool HeroChassisController::init(hardware_interface::VelocityJointInterface* effort_joint_interface, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh){

      return true;
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hero_chassis_controller");
    ros::NodeHandle nh;

    return 0;
}
