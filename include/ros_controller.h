#pragma once
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include <raisim/object/ArticulatedSystem/ArticulatedSystem.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <GaitCtrller.h>
#include <eigen3/Eigen/Eigen>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <quadruped_ctrl/QuadrupedCmdBool.h>

enum Gaits
{
  TROT = 0,
  BUNDING = 1,
  PRONKING = 2,
  STANDING = 4,
  TROT_RUN = 5,
  GALLOPING = 7,
  PACING = 8,
  WALK1 = 10,
  WALK2 = 11
};


class MPCControllerRos
{
public:
  MPCControllerRos(double freq);
  ~MPCControllerRos();

  void raisimSetup();
  void preWork();
  void readRosParams();
  void updateFeedback();
  void spin();
  void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);
  bool srvSetMode(quadruped_ctrl::QuadrupedCmdBoolRequest &req,
                  quadruped_ctrl::QuadrupedCmdBoolResponse &res);

  bool srvSetGait(quadruped_ctrl::QuadrupedCmdBoolRequest &req,
                  quadruped_ctrl::QuadrupedCmdBoolResponse &res);
private:
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::ServiceServer srv_mode_server_;
  ros::ServiceServer srv_gait_server_;
  geometry_msgs::Twist twist_;
  GaitCtrller* controller_;
  double step_freq_;
  Eigen::VectorXd q_;
  Eigen::VectorXd qd_;
  Eigen::VectorXd prev_q_;
  Eigen::VectorXd prev_qd_;
  // stand_kp  stand_kd  joint_kp  joint_kd
  std::vector<float> pid_params_;
  raisim::ArticulatedSystem* robot_;
  raisim::RaisimServer* raisim_server_;
  raisim::World world_;
  VectorNavData imu_;
  LegData legdata_;
  Eigen::VectorXd effort_;
  Eigen::VectorXd generalizedFrorce_;
  std::string urdf_path_;
};

//! @brief Шаблоннная функция для чтения параметров
template <typename T>
void readParam(const std::string param_name, T& param_value,
               const T default_value) {
  if (!ros::param::get(param_name, param_value)) {
    ROS_WARN_STREAM("Parameter \""
                    << param_name << "\" didn' find in Parameter Server."
                    << "\nSetting default value: " << &default_value);
    param_value = default_value;
  }
}

//  Изменения в направлениях осей для unitree a1
void a1_effort (Eigen::VectorXd& eff);


//  Изменения в направлениях осей для unitree a1
void a1_feedback(Eigen::VectorXd& q, Eigen::VectorXd& qd);

