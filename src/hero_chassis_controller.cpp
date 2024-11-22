//
// Created by xie on 24-11-21.
//


#include "hero_chassis_controller/hero_chassis_controller.h"
#include "pluginlib/class_list_macros.h"

namespace hero_chassis_controller
{

//回调函数实现
    void HeroChassisController::cb(hero_chassis_controller::pidConfig &config, uint32_t level){
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
    void HeroChassisController::cmdcb(const geometry_msgs::Twist::ConstPtr& msg){
      vx = msg->linear.x;
      vy = msg->linear.y;
      wz = msg->angular.z;
    }

//初始化函数实现
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

      //动态参数
      dynamic_reconfigure::Server<hero_chassis_controller::pidConfig>::CallbackType f;
      f = boost::bind(&HeroChassisController::cb, this, _1, _2);
      server.setCallback(f);

      //在配置文件中读取参数
      controller_nh.getParam("wheel_base", wheel_base);
      controller_nh.getParam("track_width", track_width);

      //cmd_sub订阅速度
      cmd_sub = controller_nh.subscribe("cmd_vel", 10, &HeroChassisController::cmdcb, this);

      return true;
    }

//状态更新函数实现
    void HeroChassisController::update(const ros::Time& time, const ros::Duration& period){
      //计算速度
      double front_left_wheel_velocity = front_left_joint_.getVelocity();
      double front_right_wheel_velocity = front_right_joint_.getVelocity();
      double back_left_wheel_velocity = back_left_joint_.getVelocity();
      double back_right_wheel_velocity = back_right_joint_.getVelocity();

      //计算力矩
      double front_left_wheel_effort = pid_front_left_.computeCommand(front_left_wheel_velocity, period);
      double front_right_wheel_effort = pid_front_right_.computeCommand(front_right_wheel_velocity, period);
      double back_left_wheel_effort = pid_back_left_.computeCommand(back_left_wheel_velocity, period);
      double back_right_wheel_effort = pid_back_right_.computeCommand(back_right_wheel_velocity, period);

      //输出力矩
      front_left_joint_.setCommand(front_left_wheel_effort);
      front_right_joint_.setCommand(front_right_wheel_effort);
      back_left_joint_.setCommand(back_left_wheel_effort);
      back_right_joint_.setCommand(back_right_wheel_effort);
    }

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}


