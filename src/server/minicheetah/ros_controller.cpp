#include <ros_controller.h>


MPCControllerRos::MPCControllerRos(double freq)
  : step_freq_(freq)
  , world_()
  //  , nh_("~")
{
  readParam<std::string>("~urdf_path", urdf_path_, "/home/den/catkin_workspaces/raisim_common/raisim_ros/src/a1_description/urdf/a1.urdf");
  raisimSetup();
  prev_q_ = Eigen::VectorXd::Zero(robot_->getGeneralizedCoordinateDim());
  prev_qd_ = VectorXd::Zero(robot_->getDOF());
    // cartesian_kp  cartesian_kd  joint_kp  joint_kd
  controller_ = new GaitCtrller(freq, std::vector<float>({100.0f, 1.0f, 0.01f, 0.05f}));// сделать рос параметрами
  generalizedFrorce_ = Eigen::VectorXd::Zero(robot_->getDOF());
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &MPCControllerRos::cmdVelCallback,this);
  srv_mode_server_ = nh_.advertiseService(ros::this_node::getName() + "/set_robot_mode", &MPCControllerRos::srvSetMode, this);
  srv_gait_server_ = nh_.advertiseService(ros::this_node::getName() + "/set_gait_type", &MPCControllerRos::srvSetGait, this);
  df_callback_type = boost::bind(&MPCControllerRos::dynamicReconfigureCallback, this, _1, _2);
  df_server.setCallback(df_callback_type);
}

MPCControllerRos::~MPCControllerRos()
{
  std::cout<<"mass "<<robot_->getMassMatrix()[0]<<std::endl;
  raisim_server_->killServer();
}

void MPCControllerRos::raisimSetup()
{
  auto binaryPath = raisim::Path("/home/den/.raisim");
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");
  // create raisim world
  world_.setTimeStep(1.0/step_freq_);
  world_.addGround();
  // COM [0.3, -0.24375, 0.0] [-1.1003e-16, 2.38132, -0.0620116]
  // inertia moments [18.3001, 28.8333, 13.8001] [1.16388e+20, 8.33528e+19, 4.95214e+19] [116388, 83352.8, 49521.4]
  // [-2.50722e-16, 1.24683e-16, -2.15625]
  raisim::Mat<3, 3> inertia({8.37, 0.0, 0.0,
                             0.0, 9.1, -0.8358,
                             0.0, -0.8358, 3.04});
  raisim::Vec<3> com({0.0, 0.34, -0.18});
  raisim::Mat<3, 3> inertia_;
  inertia_.setIdentity();
  const raisim::Vec<3> com_ = {0, 0, 0};
  auto stairs = world_.addMesh("/home/den/catkin_workspaces/raisim_common/raisim_ros/src/raisim_ros/rsc/world/stairs/solidworks/solid_stairs.obj", 1000.0, inertia, com_,0.0005);
  stairs->setPosition(0.0,3.0,0.0);
  stairs->setOrientation(0.707,0.707,0.0,0.0);
  robot_ = world_.addArticulatedSystem(urdf_path_);
  Eigen::VectorXd jointNominalConfig(robot_->getGeneralizedCoordinateDim());
  jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0,
      -0.23110604286193848, 0.7660617828369141, -1.930681824684143,
      0.2086973786354065, 0.7694507837295532, -1.945541501045227,
      -0.2868724763393402, 0.7470470666885376, -1.8848075866699219,
      0.2648811340332031, 0.7518696784973145, -1.9018760919570923;
  //  jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0,
  //      -0.23110604286193848, -0.7660617828369141, 1.930681824684143,
  //      0.2086973786354065, -0.7694507837295532, 1.945541501045227,
  //      -0.2868724763393402, -0.7470470666885376, 1.8848075866699219,
  //      0.2648811340332031, -0.7518696784973145, 1.9018760919570923;
  robot_->setGeneralizedCoordinate(jointNominalConfig);
  robot_->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

  robot_->setGeneralizedForce(Eigen::VectorXd::Zero(robot_->getDOF()));

  robot_->setName("minicheetah");
  /// launch raisim server
  raisim_server_ = new raisim::RaisimServer(&world_);
  raisim_server_->launchServer();
  raisim_server_->focusOn(robot_);
}

void MPCControllerRos::preWork()
{
  size_t iters = 10;
  controller_->SetGaitType(STANDING);
  for (size_t i = 0; i < iters; i++)
  {
    std::this_thread::sleep_for(std::chrono::microseconds(long(world_.getTimeStep() * 1000000)));
    raisim_server_->integrateWorldThreadSafe();
    robot_->getState(q_, qd_);
    a1_feedback(q_, qd_);
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

  std::vector<float> vec_q(q_.data() + 7, q_.data() + q_.size());
  std::vector<float> vec_qd(qd_.data() + 6, qd_.data() + qd_.size());
  legdata_ = LegData(vec_q, vec_qd);
}

void MPCControllerRos::spin()
{
  ros::spinOnce();
  robot_->getState(q_, qd_);
  a1_feedback(q_, qd_);
  updateFeedback();
  effort_ = controller_->TorqueCalculator(imu_, legdata_);
  a1_effort(effort_);
  generalizedFrorce_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, effort_;
  robot_->setGeneralizedForce(generalizedFrorce_);
  prev_q_ = q_;
  prev_qd_ = qd_;
  std::this_thread::sleep_for(std::chrono::microseconds(long(world_.getTimeStep() * 1000000)));
  raisim_server_->integrateWorldThreadSafe();
  controller_->SetRobotVel(twist_.linear.x, twist_.linear.y, twist_.angular.z);
}

void MPCControllerRos::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{
  twist_ = *msg;
}

bool MPCControllerRos::srvSetMode(quadruped_ctrl::QuadrupedCmdBoolRequest &req,
                                  quadruped_ctrl::QuadrupedCmdBoolResponse &res)
{
  controller_->SetRobotMode(req.cmd);
  res.result = true;
  res.description = "Service to set robot mode. 1 for low energy mode, 0 for high perfomance mode.";
  return true;
}

bool MPCControllerRos::srvSetGait(quadruped_ctrl::QuadrupedCmdBoolRequest &req,
                                  quadruped_ctrl::QuadrupedCmdBoolResponse &res)
{
  if (0 <= req.cmd && req.cmd < 12)
  {
    controller_->SetGaitType(req.cmd);
    res.result = true;
  }
  else if (req.cmd == 13)// Прыжок
  {
    controller_->jump(true);
    res.result = true;
  }
  else
    res.result = false;
  switch  (req.cmd)
  {
  case TROT:
    res.description = "Set TROT";
    break;
  case BUNDING:
    res.description = "Set BUNDING";
    break;
  case PRONKING:
    res.description = "Set PRONKING";
    break;
  case STANDING:
    res.description = "Set STANDING";
    break;
  case TROT_RUN:
    res.description = "Set TROT_RUN";
    break;
  case GALLOPING:
    res.description = "Set GALLOPING";
    break;
  case PACING:
    res.description = "Set PACING";
    break;
  case WALK1:
    res.description = "Set WALK1";
    break;
  case WALK2:
    res.description = "Set WALK2";
    break;
  default:
    res.description = "Default behavior";
  }
  return true;
}

//  Изменения в направлениях осей для unitree a1
void a1_effort (Eigen::VectorXd& eff)
{
  for (int i = 0; i < 4; i++)
  {
    eff(i * 3 + 1) = -eff(i * 3 + 1);
    eff(i * 3 + 2) = -eff(i * 3 + 2);
  }
}

//  Изменения в направлениях осей для unitree a1
void a1_feedback(Eigen::VectorXd& q, Eigen::VectorXd& qd)
{
  for (int i = 0; i < 4; i++)
  {
    q(i * 3 + 1+7) = -q(i * 3 + 1+7);
    q(i * 3 + 2+7) = -q(i * 3 + 2+7);
    qd(i * 3 + 1+6) = -qd(i * 3 + 1+6);
    qd(i * 3 + 2+6) = -qd(i * 3 + 2+6);
  }
}


void MPCControllerRos::dynamicReconfigureCallback(quadruped_msgs::generalConfig &config, uint32_t level){
  ROS_INFO("Get new config");
  // TODO: Передавать конфигу в контроллер, а там разбивать параметры
  controller_->updateConfig(config);
}
