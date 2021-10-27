#include <ros_controller.h>


MPCControllerRos::MPCControllerRos(double freq)
  : step_freq_(freq)
{
  raisimSetup();
  prev_q_ = Eigen::VectorXd::Zero(robot_->getGeneralizedCoordinateDim());
  prev_qd_ = VectorXd::Zero(robot_->getDOF());
  pid_params_ = std::vector<float>({100.0f, 1.0f, 0.01f, 0.05f});
  controller_ = new GaitCtrller(freq, pid_params_);
  controller_->SetRobotMode(0);
  generalizedFrorce_ = Eigen::VectorXd::Zero(robot_->getDOF());
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &MPCControllerRos::cmdVelCallback,this);
}

MPCControllerRos::~MPCControllerRos()
{
  raisim_server_->killServer();
}

void MPCControllerRos::raisimSetup()
{
  auto binaryPath = raisim::Path("/home/den/.raisim");
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");
  // create raisim world
  world_.setTimeStep(1.0/step_freq_);
  world_.addGround();
  robot_ = world_.addArticulatedSystem(ros::package::getPath("quadruped_ctrl") + "\\urdf\\mini_cheetah\\mini_cheetah.urdf");
  Eigen::VectorXd jointNominalConfig(robot_->getGeneralizedCoordinateDim());
  Eigen::VectorXd jointVelocityTarget(robot_->getDOF());
  jointVelocityTarget.setZero();
  jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0,
      -0.23110604286193848, -0.7660617828369141, 1.930681824684143,
      0.2086973786354065, -0.7694507837295532, 1.945541501045227,
      -0.2868724763393402, -0.7470470666885376, 1.8848075866699219,
      0.2648811340332031, -0.7518696784973145, 1.9018760919570923;
  Eigen::VectorXd jointPgain(robot_->getDOF()), jointDgain(robot_->getDOF());
  jointPgain.tail(12).setConstant(100.0);
  jointDgain.tail(12).setConstant(1.0);
  robot_->setPdGains(jointPgain, jointDgain);
  robot_->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

  robot_->setGeneralizedForce(Eigen::VectorXd::Zero(robot_->getDOF()));
  robot_->setGeneralizedCoordinate(jointNominalConfig);
  robot_->setName("minicheetah");
  /// launch raisim server
  raisim_server_ = new raisim::RaisimServer(&world_);
  raisim_server_->launchServer();
  raisim_server_->focusOn(robot_);
}

void MPCControllerRos::preWork()
{
  size_t iters = 10;
  for (size_t i = 0; i < iters; i++)
  {
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    raisim_server_->integrateWorldThreadSafe();
    robot_->getState(q_, qd_);
    updateFeedback();
    controller_->PreWork(imu_, legdata_);
    prev_q_ = q_;
    prev_qd_ = qd_;
  }
}

void MPCControllerRos::updateFeedback()
{
  tf::Matrix3x3 rot_mat(tf::Quaternion(q_(4), q_(5),q_(6), q_(3)));
  Eigen::VectorXd acc = (qd_ - prev_qd_) * step_freq_;
  acc(2) += 9.8;
  imu_.accelerometer(0, 0) = rot_mat[0][0] * acc(0) + rot_mat[1][0]*acc(1) + rot_mat[2][0] * acc(2);
  imu_.accelerometer(1, 0) = rot_mat[0][1] * acc(0) + rot_mat[1][1]*acc(1) + rot_mat[2][1] * acc(2);
  imu_.accelerometer(2, 0) = rot_mat[0][2] * acc(0) + rot_mat[1][2]*acc(1) + rot_mat[2][2] * acc(2);
  imu_.quat(3, 0) = q_(3);
  imu_.quat(0, 0) = q_(4);
  imu_.quat(1, 0) = q_(5);
  imu_.quat(2, 0) = q_(6);
  imu_.gyro(0, 0) = rot_mat[0][0] * qd_(3) + rot_mat[1][0]*qd_(4)+ rot_mat[2][0] * qd_(5);
  imu_.gyro(1, 0) = rot_mat[0][1] * qd_(3) + rot_mat[1][1]*qd_(4)+ rot_mat[2][1] * qd_(5);
  imu_.gyro(2, 0) = rot_mat[0][2] * qd_(3) + rot_mat[1][2]*qd_(4)+ rot_mat[2][2] * qd_(5);
//  imu_.gyro(0, 0) = qd_(3);
//  imu_.gyro(1, 0) = qd_(4);
//  imu_.gyro(2, 0) =qd_(5);

  std::vector<float> vec_q(q_.data() + 7, q_.data() + q_.size());
  std::vector<float> vec_qd(qd_.data() + 6, qd_.data() + qd_.size());
  legdata_ = LegData(vec_q, vec_qd);
}

void MPCControllerRos::spin()
{
  ros::spinOnce();
  robot_->getState(q_, qd_);
  updateFeedback();
  effort_ = controller_->TorqueCalculator(imu_, legdata_);

  generalizedFrorce_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, effort_;
  robot_->setGeneralizedForce(generalizedFrorce_);
  prev_q_ = q_;
  prev_qd_ = qd_;
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  raisim_server_->integrateWorldThreadSafe();
  controller_->SetRobotVel(twist_.linear.x, twist_.linear.y, twist_.angular.z);
}

void MPCControllerRos::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{
  twist_ = *msg;
}
