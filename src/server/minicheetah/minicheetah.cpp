// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <trajectory_msgs/JointTrajectory.h>
#if WIN32
#include <timeapi.h>
#endif

#include <dlfcn.h>
#include <GaitCtrller.h>


#define WORKSPACE_SRC_DIR "/home/den/catkin_workspaces/raisim_common/raisim_ros/src"
//trajectory_msgs::JointTrajectoryConstPtr points;
Eigen::VectorXd * jointNominalConfig;
Eigen::VectorXd * jointVelocityTarget;
void callback(const trajectory_msgs::JointTrajectoryConstPtr& msg)
{
  *jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0,
      msg->points.at(0).positions.at(0),
      msg->points.at(0).positions.at(1),
      msg->points.at(0).positions.at(2),

      msg->points.at(0).positions.at(3),
      msg->points.at(0).positions.at(4),
      msg->points.at(0).positions.at(5),

      msg->points.at(0).positions.at(6),
      msg->points.at(0).positions.at(7),
      msg->points.at(0).positions.at(8),

      msg->points.at(0).positions.at(9),
      msg->points.at(0).positions.at(10),
      msg->points.at(0).positions.at(11);

  *jointVelocityTarget << 0, 0, 0.0, 0.0, 0.0, 0.0,
      msg->points.at(0).velocities.at(0),
      msg->points.at(0).velocities.at(1),
      msg->points.at(0).velocities.at(2),

      msg->points.at(0).velocities.at(3),
      msg->points.at(0).velocities.at(4),
      msg->points.at(0).velocities.at(5),

      msg->points.at(0).velocities.at(6),
      msg->points.at(0).velocities.at(7),
      msg->points.at(0).velocities.at(8),

      msg->points.at(0).velocities.at(9),
      msg->points.at(0).velocities.at(10),
      msg->points.at(0).velocities.at(11);
}

trajectory_msgs::JointTrajectory createRosMsg(Eigen::VectorXd& q, Eigen::VectorXd& v)
{
  trajectory_msgs::JointTrajectory msg;
  msg.points.resize(1);
  for (int i = 7;i <= 18; i++) {
      msg.points.at(0).positions.push_back(q(i));
  }
  for (int i = 6;i <= 17; i++) {
      msg.points.at(0).velocities.push_back(v(i));
  }
  return msg;

}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "minicheetah_sim");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<trajectory_msgs::JointTrajectory>("/joint_group_position_controller/command", 100, callback);
  ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("raisim_feedback", 10);
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");
#if WIN32
  timeBeginPeriod(1); // for sleep_for function. windows default clock speed is 1/64 second. This sets it to 1ms.
#endif

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.001);

  /// create objects
  world.addGround();
    auto minicheetah = world.addArticulatedSystem(ros::package::getPath("raisim") + "\\rsc\\minicheetah\\second\\yobotics.urdf");

  /// minicheetah joint PD controller
  jointNominalConfig = new Eigen::VectorXd(minicheetah->getGeneralizedCoordinateDim());
  jointVelocityTarget = new Eigen::VectorXd(minicheetah->getDOF());

  *jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0,
    -0.23110604286193848, -0.7660617828369141, 1.930681824684143,
      0.2086973786354065, -0.7694507837295532, 1.945541501045227,
      -0.2868724763393402, -0.7470470666885376, 1.8848075866699219,
      0.2648811340332031, -0.7518696784973145, 1.9018760919570923;
//    *jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0,
//      -0.7, -1.0, 2.715,
//      0.7, -1.0, 2.715,
//      -0.7, -1.0, 2.715,
//      0.7, -1.0, 2.715;
//  *jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0,
//    0,0,0,
//    0,0,0,
//    0,0,0,
//    0,0,0;

  jointVelocityTarget->setZero();

  Eigen::VectorXd jointPgain(minicheetah->getDOF()), jointDgain(minicheetah->getDOF());
  jointPgain.tail(12).setConstant(100.0);
  jointDgain.tail(12).setConstant(1.0);

  minicheetah->setGeneralizedCoordinate(*jointNominalConfig);
  minicheetah->setGeneralizedForce(Eigen::VectorXd::Zero(minicheetah->getDOF()));
  minicheetah->setPdGains(jointPgain, jointDgain);
  minicheetah->setPdTarget(*jointNominalConfig, *jointVelocityTarget);
  minicheetah->setName("minicheetah");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(minicheetah);

  Eigen::VectorXd feedback_q;
  Eigen::VectorXd feedback_qd;

  double pid_params[4] = {100.0, 1.0, 10.0, 0.2};
  GaitCtrller mit_ctrl(500.0, pid_params);



  while (ros::ok()) {
    minicheetah->getState(feedback_q, feedback_qd);
    auto msg = createRosMsg(feedback_q, feedback_qd);
    pub.publish(msg);
    ros::spinOnce();
    minicheetah->setPdTarget(*jointNominalConfig, *jointVelocityTarget);
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    server.integrateWorldThreadSafe();

  }

  std::cout<<"mass "<<minicheetah->getMassMatrix()[0]<<std::endl;
  ROS_INFO("End program");
  for (auto it: minicheetah->getMovableJointNames())
  {
    std::cout << it << std::endl;
  }

  server.killServer();
}
