#include <franka_example_controllers/cartesian_torque_controller.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <iterator>
#include <memory>


#include <actionlib/client/simple_action_client.h>

#include <rosbag/bag.h>
#include <fstream>
#include <ostream>
#include <vector>
#include <std_msgs/Float64.h>

#include <std_msgs/Time.h>

#include <cmath>
#include <functional>
#include <memory>


#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Dense>

#include <stdexcept>
#include <string>
//#include <iomanip>

//#include <boost/type_traits/is_same.hpp>
//#include <boost/numeric/odeint/stepper/stepper_categories.hpp>
//#include <boost/numeric/odeint/integrate/null_observer.hpp>
//#include <boost/numeric/odeint/integrate/detail/integrate_adaptive.hpp>

#include <boost/numeric/odeint.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/skewness.hpp>


#include <hardware_interface/hardware_interface.h>

#include <XmlRpc.h>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <franka/robot_state.h>
#include <ros/ros.h>
#include <franka_hw/trigger_rate.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <thread>
#include <franka_example_controllers/pseudo_inversion.h>


// for UDP communication only code start

#include <mutex>
#include <thread>
#include <math.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <netdb.h>

// for UDP communication only code end


using namespace std;
using namespace Eigen; 
using namespace boost::numeric::odeint;
//using namespace plt = matplotlibcpp;

//using namespace boost::accumulators;

//typedef std::vector<double> state_type;
//typedef runge_kutta4<std::vector <double>> rk4;
//typedef runge_kutta_dopri5< std::vector<double> > stepper_type;
//typedef runge_kutta_cash_karp54<std::vector<double> > rkck54;

//std::vector<double> s(7);    // declared here for global use... sample declaration

//std::vector<double> u(7), q_share(7);     // declared here for global use
//Eigen::VectorXd H(7), tauc(7);           // declared here for global use


//ofstream fs;

//std::string filename = "exampleoutput.csv";

//double count = 0.0;
double stphase2 = 0.0;
double stphase3 = 0.0;
double stphase4 = 0.0;

namespace franka_example_controllers {

bool CartesianTorqueController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      " Initializing Energy Shaping Controller To File Now! ");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("EnergyShapingControllerToFile: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "EnergyShapingControllerToFile: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("EnergyShapingControllerToFile: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "EnergyShapingControllerToFile: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("EnergyShapingControllerToFile: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "EnergyShapingControllerToFile: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianPoseExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("EnergyShapingControllerToFile: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("EnergyShapingControllerToFile: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

//  dynamic_reconfigure_compliance_param_node_ =
//      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

//  dynamic_server_compliance_param_ = std::make_unique<
//      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

//      dynamic_reconfigure_compliance_param_node_);
  tvec_pub = node_handle.advertise<std_msgs::Float64>("Timevec", 1000);

  et_pub = node_handle.advertise<std_msgs::Float64>("et", 1000);
  eo_pub = node_handle.advertise<std_msgs::Float64>("eo", 1000);

  x_position_pub = node_handle.advertise<std_msgs::Float64>("X_Position", 1000);
  y_position_pub = node_handle.advertise<std_msgs::Float64>("Y_Position", 1000);
  z_position_pub = node_handle.advertise<std_msgs::Float64>("Z_Position", 1000);

  x_position_d_pub = node_handle.advertise<std_msgs::Float64>("X_Position_d", 1000);
  y_position_d_pub = node_handle.advertise<std_msgs::Float64>("Y_Position_d", 1000);
  z_position_d_pub = node_handle.advertise<std_msgs::Float64>("Z_Position_d", 1000);

  return true;
}
                                               

void CartesianTorqueController::starting(const ros::Time& /*time*/) {

    //initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;   // initial_pose_
    //current_time = ros::Time::now();
    //franka::RobotState robot_state = state_handle_->getRobotState();
    //initial_pose_ = robot_state.O_T_EE_d; 
    elapsed_time_ = ros::Duration(0.0);   //already exisiting format
    t_vec = ros::Duration(0.0);

}

actionlib::SimpleActionClient<franka_gripper::GraspAction> gcmd("franka_gripper/grasp", true);
//actionlib::SimpleActionClient<franka_gripper::MoveAction> gcmd1("franka_gripper/move", true);

void CartesianTorqueController::update(const ros::Time& /*time*/, const ros::Duration& period) {


  elapsed_time_ += period;
  
  double Grasp_Width;
  double Move_Width;
  double y_pos;
  double ireadit;
  std_msgs::Float64 tvec_msg;
  tvec_msg.data = elapsed_time_.toSec();
  tvec_pub.publish(tvec_msg);

  // initial values
  double kp = 200.0;            //translational stiffness constant
  double kd = 10.0;               //Damping coefficient

  std_msgs::Float64 et_msg;
  std_msgs::Float64 eo_msg;
 
  std_msgs::Float64 x_position_msg;
  std_msgs::Float64 y_position_msg;
  std_msgs::Float64 z_position_msg;

  std_msgs::Float64 x_position_d_msg;
  std_msgs::Float64 y_position_d_msg;
  std_msgs::Float64 z_position_d_msg;

  //Eigen::MatrixXd I3 = Eigen::Matrix3d::Identity();
  //Eigen::MatrixXd I6 = Eigen::MatrixXd::Identity(6,6);  
  //Eigen::MatrixXd I7 = Eigen::MatrixXd::Identity(7,7);
          
  // initial equations
  //Eigen::MatrixXd Bi = b * I7;        // I7 is equivalent to eye(7) where eye refers to identity matrix and 6 refers to size of matrix
  //Eigen::MatrixXd Ko = ko * I3;
  //Eigen::MatrixXd Kt = kt * I3; 
  //Eigen::MatrixXd Kc = 0.0 * I3;
  
  //Eigen::MatrixXd Goi = 0.5*Ko.trace()*I3 - Ko;
  //Eigen::MatrixXd Gti = 0.5*Kt.trace()*I3 - Kt;          // trace refers to tensor space operator
  //Eigen::MatrixXd Gci = 0.5*Kc.trace()*I3 - Kc;

  //double gamma = sqrt(2*epsilon);                     // square root
  //double dt = 0.001;

  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 49> mass_array = model_handle_->getMass();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data()); 
  Eigen::Map<Eigen::Matrix<double, 7, 7>> Mass(mass_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_dot(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());

  Eigen::Map<Eigen::Matrix<double, 4, 4>> H_t0(robot_state.O_T_EE.data());
  //ROS_INFO_STREAM("Initial Homogeneous Transformation Matrix: \n" 
  //                << H_t0 << "\n"); 

  double radius = 0.3;
  double angle = M_PI / 4.0 * (1.0 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
  double delta_x = radius * std::sin(angle);
  double delta_z = radius * (std::cos(angle) - 1.0);
  std::array<double, 16> H_v0_vec = robot_state.O_T_EE_d;   
 
  if (elapsed_time_.toSec() <=5.0){
  H_v0_vec[12] += delta_x;
  H_v0_vec[14] += delta_z;
  stphase2 = 1.0;
  }
  else if (elapsed_time_.toSec() > 5.0){
     if (stphase2 == 1.0){ 
        H_v0_vec[12] = robot_state.O_T_EE[12];
        H_v0_vec[14] = robot_state.O_T_EE[14];  
  
        Grasp_Width = 0.042;
  
        franka_gripper::GraspGoal goal;
        goal.width = Grasp_Width;
        goal.speed = 0.1;
        goal.force = 60;
        goal.epsilon.inner = 0.05;
        goal.epsilon.outer = 0.05;

        gcmd.sendGoal(goal);

        stphase3 = 1.0;
        stphase2 = 0.0;

     }
     else if (stphase3 == 1.0){ 
        H_v0_vec[12] = robot_state.O_T_EE[12];
        H_v0_vec[14] = robot_state.O_T_EE[14];  
        t_vec += period; 
        double angle1 = M_PI / 4.0 * (1.0 - std::cos(M_PI / 5.0 * t_vec.toSec()));
        double delta_y = radius * std::sin(angle1);
        if (t_vec.toSec() <=5.0){
           H_v0_vec[13] += delta_y;
        }
        double y_pos = robot_state.O_T_EE[13];
        double ireadit = 0.302 - y_pos;

        if (ireadit < 0.0053){ 
           H_v0_vec[13] = robot_state.O_T_EE[13];
           stphase4 = 1.0;
           stphase3 = 0.0;
        }   
     }
     else if (stphase4 == 1.0){
        H_v0_vec[12] = robot_state.O_T_EE[12];
        H_v0_vec[13] = robot_state.O_T_EE[13];
        H_v0_vec[14] = robot_state.O_T_EE[14];

        Grasp_Width = 0.032;
  
        franka_gripper::GraspGoal goal1;
        goal1.width = Grasp_Width;
        goal1.speed = 0.1;
        goal1.force = 0;
        goal1.epsilon.inner = 0.05;
        goal1.epsilon.outer = 0.05;

        gcmd.sendGoal(goal1);
        stphase4 = 0.0;
     }
  }  

  Eigen::Map<Eigen::Matrix<double, 4, 4>> H_v0(H_v0_vec.data());
  //Eigen::Map<Eigen::Matrix<double, 4, 4>> H_v0(H_v0_vec.data());

  //ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n"); 

  // H_t0 is current eef transform
  // H_t0 is read as eef current config in frame{t} wrt base frame{0}
  Eigen::MatrixXd R_t0 = H_t0.block(0,0,3,3);
  Eigen::VectorXd p_t0 = H_t0.block(0,3,3,1);

  Eigen::MatrixXd rot = R_t0;
  Eigen::VectorXd position = p_t0;

  x_position_msg.data = position[0];
  y_position_msg.data = position[1];
  z_position_msg.data = position[2];
  x_position_pub.publish(x_position_msg);
  y_position_pub.publish(y_position_msg);
  z_position_pub.publish(z_position_msg);

  Eigen::MatrixXd rot_d = H_v0.block(0,0,3,3);
  Eigen::VectorXd position_d = H_v0.block(0,3,3,1);


  x_position_d_msg.data = position_d[0];
  y_position_d_msg.data = position_d[1];
  z_position_d_msg.data = position_d[2];
  x_position_d_pub.publish(x_position_d_msg);
  y_position_d_pub.publish(y_position_d_msg);
  z_position_d_pub.publish(z_position_d_msg);

  // compute error to desired pose
  // position error
  //Eigen::Matrix<double, 6, 1> error;
  
  Eigen::VectorXd error(6,1);
  error.block(0,0,3,1) = position - position_d; 

  // orientation error
  Eigen::VectorXd vec_a1 = rot.block(0,0,3,1);
  Eigen::VectorXd vec_b1 = rot_d.block(0,0,3,1);

  Eigen::VectorXd vec1(3,1);
  vec1 << vec_a1(1,0)*vec_b1(2,0)-vec_a1(2,0)*vec_b1(1,0), vec_a1(2,0)*vec_b1(0,0)-vec_a1(0,0)*vec_b1(2,0), vec_a1(0,0)*vec_b1(1,0)-vec_a1(1,0)*vec_b1(0,0);
                         
  Eigen::VectorXd vec_a2 = rot.block(0,1,3,1);
  Eigen::VectorXd vec_b2 = rot_d.block(0,1,3,1);

  Eigen::VectorXd vec2(3,1);
  vec2 << vec_a2(1,0)*vec_b2(2,0)-vec_a2(2,0)*vec_b2(1,0), vec_a2(2,0)*vec_b2(0,0)-vec_a2(0,0)*vec_b2(2,0), vec_a2(0,0)*vec_b2(1,0)-vec_a2(1,0)*vec_b2(0,0);

  Eigen::VectorXd vec_a3 = rot.block(0,2,3,1);
  Eigen::VectorXd vec_b3 = rot_d.block(0,2,3,1);

  Eigen::VectorXd vec3(3,1);
  vec3 << vec_a3(1,0)*vec_b3(2,0)-vec_a3(2,0)*vec_b3(1,0), vec_a3(2,0)*vec_b3(0,0)-vec_a3(0,0)*vec_b3(2,0), vec_a3(0,0)*vec_b3(1,0)-vec_a3(1,0)*vec_b3(0,0);

  Eigen::VectorXd vec_sum = vec1 + vec2 + vec3;
  error.block(3,0,3,1) = vec_sum*(1/2);

  Eigen::VectorXd et_ = error.block(0,0,3,1);
  Eigen::VectorXd eo_ = error.block(3,0,3,1);

  double et = sqrt(et_(0,0)*et_(0,0) + et_(1,0)*et_(1,0) + et_(2,0)*et_(2,0));
  et_msg.data = et;               
  et_pub.publish(et_msg);
  double eo = sqrt(et_(0,0)*et_(0,0) + et_(1,0)*et_(1,0) + et_(2,0)*et_(2,0));
  eo_msg.data = eo;               
  eo_pub.publish(eo_msg);

  Eigen::VectorXd x_e(6,1);         // position and orientation error vector
  x_e.block(0,0,3,1) = et_;          // first three entries are of position error;
  x_e.block(3,0,3,1) = eo_;          // last three entries are of orientational error;  

  //Eigen::MatrixXd H_0t = Eigen::Matrix4d::Identity();
  
  //H_0t.block(0,0,3,3) = R_t0.transpose();
  //H_0t.block(0,3,3,1) = -R_t0.transpose()*p_t0;
  
  // compute error to desired pose
  // position error
//  Eigen::Matrix<double, 6, 1> error;
//  error.head(3) << position - position_d_;

  // orientation error
//  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
//    orientation.coeffs() << -orientation.coeffs();
//  }

  // "difference" quaternion
//  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
//  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
//  error.tail(3) << -transform.rotation() * error.tail(3);

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
//  tau_task << jacobian.transpose() *
//                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  tau_task << jacobian.transpose() *
                  (-kp * x_e - kd * (jacobian * q_dot));


  // nullspace PD control with damping ratio = 1
//  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
//                    jacobian.transpose() * jacobian_transpose_pinv) *
//                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
//                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
//  tau_d << tau_task + tau_nullspace + coriolis;
  tau_d << tau_task + coriolis;

  // Saturate torque rate to avoid discontinuities
//  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
}


}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianTorqueController,
                       controller_interface::ControllerBase)
