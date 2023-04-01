#include <franka_example_controllers/energy_shaping_controller_to_file.h>

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

double count = 0.0;
double stph2 = 0.0;
double stph3 = 0.0;
double stph4 = 0.0;

std::vector<double> ftleft(7);

double s0 = sqrt(2*10);
double s1 = sqrt(2*10);
double s2 = sqrt(2*10);
double s3 = sqrt(2*10);
double s4 = sqrt(2*10);
double s5 = sqrt(2*10);
double s6 = sqrt(2*10); 

double s_prev0 = sqrt(2*10);
double s_prev1 = sqrt(2*10);
double s_prev2 = sqrt(2*10);
double s_prev3 = sqrt(2*10);
double s_prev4 = sqrt(2*10);
double s_prev5 = sqrt(2*10);
double s_prev6 = sqrt(2*10); 

namespace franka_example_controllers {

bool EnergyShapingControllerToFile::init(hardware_interface::RobotHW* robot_hw,
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

  energy_pub = node_handle.advertise<std_msgs::Float64>("Energy", 1000);
  lambda_pub = node_handle.advertise<std_msgs::Float64>("lambda", 1000);

  power_publ = node_handle.advertise<std_msgs::Float64>("Power_l", 1000);
  power_pubr = node_handle.advertise<std_msgs::Float64>("Power_r", 1000);
  power_pub = node_handle.advertise<std_msgs::Float64>("Power", 1000);
  beta_pub = node_handle.advertise<std_msgs::Float64>("beta", 1000);
  et_pub = node_handle.advertise<std_msgs::Float64>("et", 1000);
  eo_pub = node_handle.advertise<std_msgs::Float64>("eo", 1000);

  x_position_pub = node_handle.advertise<std_msgs::Float64>("X_Position", 1000);
  y_position_pub = node_handle.advertise<std_msgs::Float64>("Y_Position", 1000);
  z_position_pub = node_handle.advertise<std_msgs::Float64>("Z_Position", 1000);

  x_position_d_pub = node_handle.advertise<std_msgs::Float64>("X_Position_d", 1000);
  y_position_d_pub = node_handle.advertise<std_msgs::Float64>("Y_Position_d", 1000);
  z_position_d_pub = node_handle.advertise<std_msgs::Float64>("Z_Position_d", 1000);

  H0_pub = node_handle.advertise<std_msgs::Float64>("Level_ET1", 1000);
  H1_pub = node_handle.advertise<std_msgs::Float64>("Level_ET2", 1000);
  H2_pub = node_handle.advertise<std_msgs::Float64>("Level_ET3", 1000);
  H3_pub = node_handle.advertise<std_msgs::Float64>("Level_ET4", 1000);
  H4_pub = node_handle.advertise<std_msgs::Float64>("Level_ET5", 1000);
  H5_pub = node_handle.advertise<std_msgs::Float64>("Level_ET6", 1000);
  H6_pub = node_handle.advertise<std_msgs::Float64>("Level_ET7", 1000);

  return true;
}
                                               

void EnergyShapingControllerToFile::starting(const ros::Time& /*time*/) {

    //initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;   // initial_pose_
    //current_time = ros::Time::now();
    //franka::RobotState robot_state = state_handle_->getRobotState();
    //initial_pose_ = robot_state.O_T_EE_d; 
    elapsed_time_ = ros::Duration(0.0);   //already exisiting format
    t_vec = ros::Duration(0.0);

}

actionlib::SimpleActionClient<franka_gripper::GraspAction> ac("franka_gripper/grasp", true);
actionlib::SimpleActionClient<franka_gripper::MoveAction> ac1("franka_gripper/move", true);

void EnergyShapingControllerToFile::update(const ros::Time& /*time*/, const ros::Duration& period) {


  elapsed_time_ += period;
  
  double Grasp_Width;
  double Move_Width;
  double y_pos;
  double ireadit;
  std_msgs::Float64 tvec_msg;
  tvec_msg.data = elapsed_time_.toSec();
  tvec_pub.publish(tvec_msg);

  // initial values
  double ko = 320.0;             //rotational stiffness constant
  double kt = 2000.0;            //translational stiffness constant
  double b = 10.0;               //Damping coefficient
  double epsilon = 0.001;      //Minimum energy in tank
  double Emax = 0.6;             //Maximum allowed energy
  double Pmax = 0.8;             //Maximum allowed power

  std_msgs::Float64 energy_msg;
  std_msgs::Float64 lambda_msg;
  std_msgs::Float64 power_msg;
  std_msgs::Float64 beta_msg;
  std_msgs::Float64 et_msg;
  std_msgs::Float64 eo_msg;

  std_msgs::Float64 power_msgl;
  std_msgs::Float64 power_msgr;
 
  std_msgs::Float64 x_position_msg;
  std_msgs::Float64 y_position_msg;
  std_msgs::Float64 z_position_msg;

  std_msgs::Float64 x_position_d_msg;
  std_msgs::Float64 y_position_d_msg;
  std_msgs::Float64 z_position_d_msg;

  std_msgs::Float64 H0_msg;
  std_msgs::Float64 H1_msg;
  std_msgs::Float64 H2_msg;
  std_msgs::Float64 H3_msg;
  std_msgs::Float64 H4_msg;
  std_msgs::Float64 H5_msg;
  std_msgs::Float64 H6_msg;
  

  Eigen::MatrixXd I3 = Eigen::Matrix3d::Identity();
  Eigen::MatrixXd I6 = Eigen::MatrixXd::Identity(6,6);  
  Eigen::MatrixXd I7 = Eigen::MatrixXd::Identity(7,7);
          
  // initial equations
  Eigen::MatrixXd Bi = b * I7;        // I7 is equivalent to eye(7) where eye refers to identity matrix and 6 refers to size of matrix
  Eigen::MatrixXd Ko = ko * I3;
  Eigen::MatrixXd Kt = kt * I3; 
  Eigen::MatrixXd Kc = 0.0 * I3;
  
  Eigen::MatrixXd Goi = 0.5*Ko.trace()*I3 - Ko;
  Eigen::MatrixXd Gti = 0.5*Kt.trace()*I3 - Kt;          // trace refers to tensor space operator
  Eigen::MatrixXd Gci = 0.5*Kc.trace()*I3 - Kc;

  double gamma = sqrt(2*epsilon);                     // square root
  double dt = 0.001;

  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
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
  //Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
  //    robot_state.tau_J_d.data());

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
  stph2 = 1.0;

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

  Eigen::MatrixXd H_0t = Eigen::Matrix4d::Identity();
  
  H_0t.block(0,0,3,3) = R_t0.transpose();
  H_0t.block(0,3,3,1) = -R_t0.transpose()*p_t0;
   
  Eigen::Matrix4d H_vt = H_0t * H_v0;             
    
  // extracting rotaional Rvt and translational pvt part from Hvt for further calculating wrench 
  Eigen::Matrix3d R_vt = H_vt.block(0,0,3,3);

  Eigen::Vector3d p_vt = H_vt.block(0,3,3,1);

  Eigen::Matrix3d R_tv = R_vt.transpose(); 

  // converting position vector to skew-symmetric matrix
  //MatrixXd tilde_p_vt = [0 -p_vt(3,1) p_vt(2,1);p_vt(3,1) 0 -p_vt(1,1);-p_vt(2,1) p_vt(1,1) 0];
  Eigen::Matrix3d tilde_p_vt(3,3); 
  tilde_p_vt << 0,-p_vt(2,0),p_vt(1,0),p_vt(2,0),0,-p_vt(0,0),-p_vt(1,0),p_vt(0,0),0;
            
                        
  // Safety layer (PE, KE and total Energy of the system)
  double Vti = (-0.25*(tilde_p_vt*Gti*tilde_p_vt).trace())-(0.25*(tilde_p_vt*R_vt*Gti*R_tv*tilde_p_vt).trace());
  double Voi = -((Goi*R_vt).trace());
  double Vci = ((Gci*R_tv*tilde_p_vt).trace());


  double V_pi = Vti + Voi + Vci + 480.0;         // initial potential energy

  double T_k = 0.5 * q_dot.transpose() * Mass * q_dot;        // transpose of qdot x M x qdot

  double E_tot = T_k + V_pi;               // initial energy of the system
 
  double lamb_da;

  if (E_tot > Emax){  
      lamb_da = (Emax - T_k)/ V_pi;
  }else{
      lamb_da = 1.0;
  } 
  //stiff_vec.push_back(lamb_da);
  lambda_msg.data = lamb_da;               
  lambda_pub.publish(lambda_msg);

  // calculation of new co-stiffness matrices and corresponding potential energy
  Eigen::MatrixXd Go = lamb_da * Goi;           // new co-stiffness matrices
  Eigen::MatrixXd Gt = lamb_da * Gti;
  Eigen::MatrixXd Gc = lamb_da * Gci;
     
  double Vt = (-0.25*(tilde_p_vt*Gt*tilde_p_vt).trace())-(0.25*(tilde_p_vt*R_vt*Gt*R_tv*tilde_p_vt).trace());
  double Vo = -((Go*R_vt).trace());
  double Vc = ((Gc*R_tv*tilde_p_vt).trace());


  double V_p = Vt + Vo + Vc + 480.0;         // potential energy
  E_tot = T_k + lamb_da*V_pi;            // total energy of the system
  energy_msg.data = E_tot;               
  energy_pub.publish(energy_msg);

  // Rotational part of wrench
  Eigen::MatrixXd tilde_m_t = - 2.0* 0.5*(Go*R_vt-(Go*R_vt).transpose())-0.5*(Gt*R_tv*tilde_p_vt*tilde_p_vt*R_vt-    (Gt*R_tv*tilde_p_vt*tilde_p_vt*R_vt).transpose())-2.0*0.5*(Gc*tilde_p_vt*R_vt-(Gc*tilde_p_vt*R_vt).transpose());
  Eigen::VectorXd m_t(3,1); 
  //m_t = [tilde_m_t(3,2); tilde_m_t(1,3); tilde_m_t(2,1)];   in matlab
  m_t << tilde_m_t(2,1),tilde_m_t(0,2),tilde_m_t(1,0);

  // Translational part of wrench
  Eigen::MatrixXd tilde_f_t = -R_tv * 0.5*(Gt*tilde_p_vt- (Gt*tilde_p_vt).transpose())*R_vt-0.5*(Gt*R_tv*tilde_p_vt*R_vt-(Gt*R_tv*tilde_p_vt*R_vt).transpose())-2.0*0.5*(Gc*R_vt-(Gc*R_vt).transpose());
  Eigen::VectorXd f_t(3,1);
  //f_t = [tilde_f_t(3,2); tilde_f_t(1,3); tilde_f_t(2,1)];   in matlab
  f_t << tilde_f_t(2,1),tilde_f_t(0,2),tilde_f_t(1,0);
            
  Eigen::VectorXd Wt(6,1);           // wrench vector initialization
  //Wt << 0.0,0.0,0.0,0.0,0.0,0.0; 
    
  Wt.block(0,0,3,1) = f_t;          // Wsn0(1:3,1) = t ... t represented with small m here;
  Wt.block(3,0,3,1) = m_t;          // Wsn0(4:6,1) = f; 

  Eigen::MatrixXd R_0t = H_0t.block(0,0,3,3);
  //p_0t = H_0t.block(0,3,3,1);

  //MatrixXd tilde_p_0t(3,3);
  //tilde_p_0t << 0,-p_0t(2,0),p_0t(1,0),p_0t(2,0),0,-p_0t(0,0),-p_0t(1,0),p_0t(0,0),0;
  Eigen::MatrixXd tilde_p_t0(3,3); 
  tilde_p_t0 << 0.0,-p_t0(2,0),p_t0(1,0),p_t0(2,0),0.0,-p_t0(0,0),-p_t0(1,0),p_t0(0,0),0.0;

  Eigen::MatrixXd AdjT_H_t0(6,6);
  //AdjT_H_t0.fill(0.0);

  AdjT_H_t0.block(0,0,3,3) = R_0t;
  AdjT_H_t0.block(0,3,3,3) = -R_0t*tilde_p_t0;
  AdjT_H_t0.block(3,0,3,3) = 0.0 * I3;
  AdjT_H_t0.block(3,3,3,3) = R_0t;

  Eigen::VectorXd W0(6,1);           // wrench vector transformation of 
  W0 = -AdjT_H_t0* Wt;     

/*    // data received from F/T sensor is stored here for use in controller maths starting
//  std::vector<double> ftleft(7);
    //UDP connection setup
    
  struct sockaddr_in cpp_addr, matlab_addr, sender_addr;

    // C++ UDP Port

  cpp_addr.sin_family = AF_INET;
  cpp_addr.sin_addr.s_addr = inet_addr("172.16.0.1");   //10.11.104.24
  cpp_addr.sin_port = htons(16385);


    // Matlab UDP Port

  matlab_addr.sin_family = AF_INET;
  matlab_addr.sin_addr.s_addr = inet_addr("172.16.0.3");
  matlab_addr.sin_port = htons(8080);

    // Create and Bind socket to local address

  client = socket(AF_INET, SOCK_DGRAM, 0);  

    if (client < 0)
    {
       std::cout << "Error creating socket" << std::endl;
       exit(1);
    }

    std::cout << "Socket created" << std::endl;

    if ((bind(client, (struct sockaddr*)&cpp_addr, sizeof(cpp_addr)) ) < 0)
    {
       std::cout << "Error binding connection" << std::endl;
       exit(1);

    }  

    int recvlen;
    int bufSize = 2048;
    unsigned char buf[bufSize];
    socklen_t addrlen = sizeof(matlab_addr);

    //double w = 0.0;

    //char const * message;

    std::cout << "Waiting to receive" << std::endl;

    recvlen = recvfrom(client, buf, bufSize, 0, (struct sockaddr *)&sender_addr, &addrlen);
    //recvlen = recvfrom(client, buf, bufSize, 0, &addrlen);
 
    if (recvlen > 0) {
       buf[recvlen] = 0;
       exit(1);
    }else{
       std::cout << "Nothing received" << std::endl;
       exit(1);
    } 

    double (* bufDouble) = reinterpret_cast<double *>(buf);

   std::cout << "Received is" << bufDouble[0] << " "
                              << bufDouble[1] << " "
                              << bufDouble[2] << " "
                              << bufDouble[3] << " "
                              << bufDouble[4] << " "
                              << bufDouble[5] << std::endl;
*/
  // Power of the System 
     
  double P_c = ((jacobian.transpose() * W0 - Bi * q_dot).transpose()) * q_dot ;  // Initial power of the controller
   
  double beta;
  if (P_c > Pmax){ 
  
        beta = ((((jacobian.transpose() * W0).transpose())*q_dot) - Pmax)/ ((q_dot.transpose())*Bi*q_dot);
  }else{
        beta = 1.0; 
  }
  beta_msg.data = beta;               
  beta_pub.publish(beta_msg);


  // New joint damping matrix using scaling parameter beta
  Eigen::MatrixXd B = beta * Bi;

  // New power of the controller using new joint damping matrix
  Eigen::VectorXd tau_cmd(7);
  tau_cmd = (jacobian.transpose()) * W0 - B * q_dot;     // Controller force

  P_c = tau_cmd.transpose() * q_dot ;      // Power of the controller
  power_msg.data = P_c;    
  power_pub.publish(power_msg);

  std::vector<double> u(7), s(7);      
  Eigen::VectorXd H(7);
  Eigen::VectorXd tauc(7);           

  //state_type integrate_adaptive();
  //integrate_adaptive( rk4() ,my_system, s0 , t0 , t1 , d_t,my_observer);   
  //integrate_adaptive( rkck54() , my_system , s0 , t0 , t1 , d_t, my_observer);   

  //solver to integrate states using integrate_adaptive();
  //integrate_adaptive( make_controlled(1E-12,1E-12,stepper_type()),
  //                    my_system, s0, t0 , t1 , d_t, my_observer ); 
    
  //integrate(my_system, s0, t0 , t1 , d_t);
  
  //ROS_INFO_STREAM("Energy Tanks States : \n" 
  //                << " " << s0[0] << " " << s0[1] << " " << s0[2] << " " << s0[3] << " " << s0[4] << " " << s0[5] << " " << s0[6] << "\n"); 
   
  // adding previous value to the integrated/new state   
  //std::transform(s0.begin(),s0.end(),s0_in.begin(),
  //               s0.begin(),std::plus<double>());
     
  // Controller torques for each joint
  tauc[0] = tau_cmd[0];
  tauc[1] = tau_cmd[1];
  tauc[2] = tau_cmd[2];
  tauc[3] = tau_cmd[3];
  tauc[4] = tau_cmd[4];
  tauc[5] = tau_cmd[5];
  tauc[6] = tau_cmd[6];

  // Energy in each tank, an energy tank is modeled as a spring with const stiffness k = 1
  // connected to robot through a transmission unit 
  // so H = 0.5*k*s^2 ==> H = 0.5*s^2 
  H[0] = 0.5*s0*s0;
  H0_msg.data = H[0];    
  H0_pub.publish(H0_msg);

  H[1] = 0.5*s1*s1;
  H1_msg.data = H[1];    
  H1_pub.publish(H1_msg);

  H[2] = 0.5*s2*s2;
  H2_msg.data = H[2];    
  H2_pub.publish(H2_msg);

  H[3] = 0.5*s3*s3;
  H3_msg.data = H[3];    
  H3_pub.publish(H3_msg);

  H[4] = 0.5*s4*s4;
  H4_msg.data = H[4];    
  H4_pub.publish(H4_msg);

  H[5] = 0.5*s5*s5;
  H5_msg.data = H[5];    
  H5_pub.publish(H5_msg);

  H[6] = 0.5*s6*s6;         
  H6_msg.data = H[6];    
  H6_pub.publish(H6_msg);

  ROS_INFO_STREAM("Energy Tanks Last State: \n" << elapsed_time_ << " " << s_prev0 << " " << s_prev1 << " " << s_prev2 << " " << s_prev3 << " " << s_prev4 << " " << s_prev5 << " " << s_prev6 << "\n");  


  // transmission unit allows power flow from controller to robot and it is regulated by ratio u
  // here u is transmission variable
  if (H[0] > epsilon){    
       u[0] = -tauc[0]/s0;
       s0 = s_prev0 + (-tauc[0]/s_prev0) * q_dot[0] * dt;
  }else{
       u[0] = -tauc[0]/pow(gamma,2)*s0;
       s0 = s_prev0 + ((-tauc[0]/gamma)/gamma)*s_prev0 * q_dot[0] * dt;
  }

  if (H[1] > epsilon){   
       u[1] = -tauc[1]/s1;
       s1 = s_prev1 + (-tauc[1]/s_prev1) * q_dot[1] * dt;
  }else{
       u[1] = -tauc[1]/pow(gamma,2)*s1;
       s1 = s_prev1 + ((-tauc[1]/gamma)/gamma)*s_prev1 * q_dot[1] * dt;
  }

  if (H[2] > epsilon){  
       u[2] = -tauc[2]/s2;
       s2 = s_prev2 + (-tauc[2]/s_prev2) * q_dot[2] * dt; 
  }else{
       u[2] = -tauc[2]/pow(gamma,2)*s2;
       s2 = s_prev2 + ((-tauc[2]/gamma)/gamma)*s_prev2 * q_dot[2] * dt; 
  }

  if (H[3] > epsilon){   
       u[3] = -tauc[3]/s3;
       s3 = s_prev3 + (-tauc[3]/s_prev3) * q_dot[3] * dt; 
  }else{
       u[3] = -tauc[3]/pow(gamma,2)*s3;
       s3 = s_prev3 + ((-tauc[3]/gamma)/gamma)*s_prev3 * q_dot[3] * dt; 
  }

  if (H[4] > epsilon){   
       u[4] = -tauc[4]/s4;
       s4 = s_prev4 + (-tauc[4]/s_prev4) * q_dot[4] * dt; 
  }else{
       u[4] = -tauc[4]/pow(gamma,2)*s4; 
       s4 = s_prev4 + ((-tauc[4]/gamma)/gamma)*s_prev4 * q_dot[4] * dt; 
  }

  if (H[5] > epsilon){  
       u[5] = -tauc[5]/s5;
       s5 = s_prev5 + (-tauc[5]/s_prev5) * q_dot[5] * dt; 
  }else{
       u[5] = -tauc[5]/pow(gamma,2)*s5;
       s5 = s_prev5 + ((-tauc[5]/gamma)/gamma)*s_prev5 * q_dot[5] * dt; 
  }

  if (H[6] > epsilon){ 
       u[6] = -tauc[6]/s6;
       s6 = s_prev6 + (-tauc[6]/s_prev6) * q_dot[6] * dt;
  }else{
       u[6] = -tauc[6]/pow(gamma,2)*s6;
       s6 = s_prev6 + ((-tauc[6]/gamma)/gamma)*s_prev6 * q_dot[6] * dt;
  }

  

  ROS_INFO_STREAM("Energy Tanks States Updated: \n" 
                 << elapsed_time_ << " " << s0 << " " << s1 << " " << s2 << " " << s3 << " " << s4 << " " << s5 << " " << s6 << "\n"); 

  s[0] = s0;
  s[1] = s1;
  s[2] = s2;
  s[3] = s3;
  s[4] = s4;
  s[5] = s5;
  s[6] = s6;

  s_prev0 = s0;
  s_prev1 = s1;
  s_prev2 = s2;
  s_prev3 = s3;
  s_prev4 = s4;
  s_prev5 = s5;
  s_prev6 = s6; 

  //std::vector<double> check_sum{};
  //check_sum.push_back(1.0); 
  //double lets_see = accumulate(check_sum.begin(), check_sum.end(),1);    
  //check_sum.push_back(1);  
  //check_sum.resize(check_sum.size()+1,dt);


  //double t = ros::Time::now().toSec();
  //t += period.toSec();
  //t += period;    // INFINITY    10;
  //double d_t = 0.001; 
  //double t1 = 0.0;
  //t1 = t + d_t;

  //integrate(my_system, s, t , t1 , d_t);
//  ROS_INFO_STREAM("Derivatives : \n" 
//                 << elapsed_time_ << " " << dsdt[0] << " " << dsdt[1] << " " << dsdt[2] << " " << dsdt[3] << " " << dsdt[4] << " " << dsdt[5] << " " << dsdt[6] << "\n");

  
  ROS_INFO_STREAM("Energy for each joint : \n" << elapsed_time_ << " " << H[0] << " " << H[1] << " " << H[2] << " " << H[3] << " " << H[4] << " " << H[5] << " " << H[6] << "\n"); 

  ROS_INFO_STREAM("Energy  : " << " " << E_tot << "\n");

  ROS_INFO_STREAM("Lambda  : " << " " << lamb_da << "\n");

  ROS_INFO_STREAM("Power : " << " " << P_c << "\n");

  ROS_INFO_STREAM("Beta : " << " " << beta << "\n");

//  ROS_INFO_STREAM("Error Position & Orientation: \n" 
//                 << elapsed_time_ << " " << error[0] << " " << error[1] << " " << error[2] << " " << error[3] << " " << error[4] << " " << error[5] << " " << "\n"); 

  ROS_INFO_STREAM("Error : " << " " << et << " " << eo << "\n");
  
  Eigen::VectorXd tau_d(7); 
  //tau_d.setZero(); 
  for (size_t i= 0;  i< 7; ++i) {
       tau_d[i] = -u[i] * s[i];   
       joint_handles_[i].setCommand(tau_d(i));
  }
  ROS_INFO_STREAM("Torques for robot joints : \n"  << " " << tau_d[0] << " " << tau_d[1] << " " << tau_d[2] << " " << tau_d[3] << " " << tau_d[4] << " " << tau_d[5] << " " << tau_d[6] << "\n");  
  
  //ROS_INFO_STREAM("Count : " << " " << count << "\n");
  //++count;
}

  else if (elapsed_time_.toSec() > 5.0){
     if (stph2 == 1.0){ 
        H_v0_vec[12] = robot_state.O_T_EE[12];
        H_v0_vec[14] = robot_state.O_T_EE[14];  
  
        Grasp_Width = 0.042;
  
        franka_gripper::GraspGoal goal;
        goal.width = Grasp_Width;
        goal.speed = 0.1;
        goal.force = 60;
        goal.epsilon.inner = 0.05;
        goal.epsilon.outer = 0.05;

        ac.sendGoal(goal);

        stph3 = 1.0;
        stph2 = 0.0;

     }
     else if (stph3 == 1.0){ 
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
           stph4 = 1.0;
           stph3 = 0.0;
        }   
     }
     else if (stph4 == 1.0){
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

        ac.sendGoal(goal1);
        stph4 = 0.0;
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

  //left human contact x = 14 cm y = 15.75 cm z = unchanged
  //making homogeneous matrix, now H_tl is wrt end effector and so 
  //converting wrt to base frame as H_0l)
  Eigen::MatrixXd H_l0 = H_t0;    // using rot matrix of eef wrt intertial frame and change position vector
  Eigen::Vector2d pt_lt;
  pt_lt << 0.6469,0.1575;
  H_l0.block(0,3,2,1) = pt_lt;

  //right human contact x = 14 cm y = 15.75 cm z = unchanged
  //state  (making homegeneous matrix, H_tr is wrt end effector and so 
  //converting wrt to base frameas H_0r)
  Eigen::MatrixXd  H_r0= H_t0; // using rot matrix of eef wrt intertial frame and change position vector
  Eigen::Vector2d pt_rt;
  pt_rt << 0.6469,-0.1575;
  H_r0.block(0,3,2,1) = pt_rt; 

  Eigen::MatrixXd H_0t = Eigen::Matrix4d::Identity();
  
  H_0t.block(0,0,3,3) = R_t0.transpose();
  H_0t.block(0,3,3,1) = -R_t0.transpose()*p_t0;
   
  Eigen::Matrix4d H_vt = H_0t * H_v0;             
    
  // extracting rotaional Rvt and translational pvt part from Hvt for further calculating wrench 
  Eigen::Matrix3d R_vt = H_vt.block(0,0,3,3);

  Eigen::Vector3d p_vt = H_vt.block(0,3,3,1);

  Eigen::Matrix3d R_tv = R_vt.transpose(); 

  // converting position vector to skew-symmetric matrix
  //MatrixXd tilde_p_vt = [0 -p_vt(3,1) p_vt(2,1);p_vt(3,1) 0 -p_vt(1,1);-p_vt(2,1) p_vt(1,1) 0];
  Eigen::Matrix3d tilde_p_vt(3,3); 
  tilde_p_vt << 0,-p_vt(2,0),p_vt(1,0),p_vt(2,0),0,-p_vt(0,0),-p_vt(1,0),p_vt(0,0),0;
            
                        
  // Safety layer (PE, KE and total Energy of the system)
  double Vti = (-0.25*(tilde_p_vt*Gti*tilde_p_vt).trace())-(0.25*(tilde_p_vt*R_vt*Gti*R_tv*tilde_p_vt).trace());
  double Voi = -((Goi*R_vt).trace());
  double Vci = ((Gci*R_tv*tilde_p_vt).trace());

  double V_pi = Vti + Voi + Vci + 480.0;         // initial potential energy

  double T_k = 0.5 * q_dot.transpose() * Mass * q_dot;        // transpose of qdot x M x qdot

  double E_left = T_k + V_pi;
  double E_right = T_k + V_pi;
  double E_tot = T_k + V_pi;               // initial energy of the system
 
  double lamb_da;

  if (E_left > Emax || E_right > Emax || E_tot > Emax){  
      lamb_da = (Emax - T_k)/ V_pi;
  }else{
      lamb_da = 1.0;
  } 
  //stiff_vec.push_back(lamb_da);
  lambda_msg.data = lamb_da;               
  lambda_pub.publish(lambda_msg);

  // calculation of new co-stiffness matrices and corresponding potential energy
  Eigen::MatrixXd Go = lamb_da * Goi;           // new co-stiffness matrices
  Eigen::MatrixXd Gt = lamb_da * Gti;
  Eigen::MatrixXd Gc = lamb_da * Gci;
     
  double Vt = (-0.25*(tilde_p_vt*Gt*tilde_p_vt).trace())-(0.25*(tilde_p_vt*R_vt*Gt*R_tv*tilde_p_vt).trace());
  double Vo = -((Go*R_vt).trace());
  double Vc = ((Gc*R_tv*tilde_p_vt).trace());


  double V_p = Vt + Vo + Vc + 480.0;         // potential energy
  E_left = T_k + lamb_da*V_pi;
  E_right = T_k + lamb_da*V_pi;
  E_tot = T_k + lamb_da*V_pi;            // total energy of the system
  energy_msg.data = E_tot;               
  energy_pub.publish(energy_msg);

  // Rotational part of wrench
  Eigen::MatrixXd tilde_m_t = - 2.0* 0.5*(Go*R_vt-(Go*R_vt).transpose())-0.5*(Gt*R_tv*tilde_p_vt*tilde_p_vt*R_vt-    (Gt*R_tv*tilde_p_vt*tilde_p_vt*R_vt).transpose())-2.0*0.5*(Gc*tilde_p_vt*R_vt-(Gc*tilde_p_vt*R_vt).transpose());
  Eigen::VectorXd m_t(3,1); 
  //m_t = [tilde_m_t(3,2); tilde_m_t(1,3); tilde_m_t(2,1)];   in matlab
  m_t << tilde_m_t(2,1),tilde_m_t(0,2),tilde_m_t(1,0);

  // Translational part of wrench
  Eigen::MatrixXd tilde_f_t = -R_tv * 0.5*(Gt*tilde_p_vt- (Gt*tilde_p_vt).transpose())*R_vt-0.5*(Gt*R_tv*tilde_p_vt*R_vt-(Gt*R_tv*tilde_p_vt*R_vt).transpose())-2.0*0.5*(Gc*R_vt-(Gc*R_vt).transpose());
  Eigen::VectorXd f_t(3,1);
  //f_t = [tilde_f_t(3,2); tilde_f_t(1,3); tilde_f_t(2,1)];   in matlab
  f_t << tilde_f_t(2,1),tilde_f_t(0,2),tilde_f_t(1,0);
            
  Eigen::VectorXd Wt(6,1);           // wrench vector initialization
  //Wt << 0.0,0.0,0.0,0.0,0.0,0.0; 
    
  Wt.block(0,0,3,1) = f_t;          // Wsn0(1:3,1) = t ... t represented with small m here;
  Wt.block(3,0,3,1) = m_t;          // Wsn0(4:6,1) = f; 

  Eigen::MatrixXd R_0t = H_0t.block(0,0,3,3);
  //p_0t = H_0t.block(0,3,3,1);

  //MatrixXd tilde_p_0t(3,3);
  //tilde_p_0t << 0,-p_0t(2,0),p_0t(1,0),p_0t(2,0),0,-p_0t(0,0),-p_0t(1,0),p_0t(0,0),0;
  Eigen::MatrixXd tilde_p_t0(3,3); 
  tilde_p_t0 << 0.0,-p_t0(2,0),p_t0(1,0),p_t0(2,0),0.0,-p_t0(0,0),-p_t0(1,0),p_t0(0,0),0.0;

  Eigen::MatrixXd AdjT_H_t0(6,6);
  //AdjT_H_t0.fill(0.0);

  AdjT_H_t0.block(0,0,3,3) = R_0t;
  AdjT_H_t0.block(0,3,3,3) = -R_0t*tilde_p_t0;
  AdjT_H_t0.block(3,0,3,3) = 0.0 * I3;
  AdjT_H_t0.block(3,3,3,3) = R_0t;

  Eigen::Vector3d pt_rob0 = H_t0.block(0,3,3,1);

  Eigen::Vector3d pt_l0 = H_l0.block(0,3,3,1);

  Eigen::Vector3d pt_r0 = H_r0.block(0,3,3,1);

  Eigen::Vector3d pt_lrob = pt_l0 - pt_rob0;  // position vector from eef to left contact point

  Eigen::Vector3d pt_rrob = pt_r0 - pt_rob0;  // position vector from eef to right contact point

  Eigen::MatrixXd tilde_pt_lrob(3,3); 
  tilde_pt_lrob << 0.0,-pt_lrob(2,0),pt_lrob(1,0),pt_lrob(2,0),0.0,-pt_lrob(0,0),-pt_lrob(1,0),pt_lrob(0,0),0.0;

  Eigen::MatrixXd tilde_pt_rrob(3,3); 
  tilde_pt_rrob << 0.0,-pt_rrob(2,0),pt_rrob(1,0),pt_rrob(2,0),0.0,-pt_rrob(0,0),-pt_rrob(1,0),pt_rrob(0,0),0.0;

  Eigen::MatrixXd Gl(6,6);
  Gl.block(0,0,3,3) = I3;
  Gl.block(0,3,3,3) = 0.0 * I3;
  Gl.block(3,0,3,3) = tilde_pt_lrob;
  Gl.block(3,3,3,3) = I3;

  Eigen::MatrixXd Gr(6,6);
  Gr.block(0,0,3,3) = I3;
  Gr.block(0,3,3,3) = 0.0 * I3;
  Gr.block(3,0,3,3) = -tilde_pt_rrob;
  Gr.block(3,3,3,3) = I3;

  CompleteOrthogonalDecomposition<MatrixXd> codl(Gl);
  MatrixXd pinv_Gl = codl.pseudoInverse();

  CompleteOrthogonalDecomposition<MatrixXd> codr(Gr);
  MatrixXd pinv_Gr = codr.pseudoInverse();  

  Eigen::VectorXd W0_l(6,1);
  W0_l   = -AdjT_H_t0 * (I6 - pinv_Gl*(Gl))*Wt;

  Eigen::VectorXd W0_r(6,1);
  W0_r   = -AdjT_H_t0 * (I6 - pinv_Gr*(Gr))*Wt;

  Eigen::VectorXd W_c(6,1);
  W_c = W0_l + W0_r ;

  Eigen::VectorXd W0(6,1);           // wrench vector transformation of 
  W0 = W_c + (-AdjT_H_t0 * Wt);
/*    // data received from F/T sensor is stored here for use in controller maths starting
//  std::vector<double> ftleft(7);
    //UDP connection setup
    
  struct sockaddr_in cpp_addr, matlab_addr, sender_addr;

    // C++ UDP Port

  cpp_addr.sin_family = AF_INET;
  cpp_addr.sin_addr.s_addr = inet_addr("172.16.0.1");   //10.11.104.24
  cpp_addr.sin_port = htons(16385);


    // Matlab UDP Port

  matlab_addr.sin_family = AF_INET;
  matlab_addr.sin_addr.s_addr = inet_addr("172.16.0.3");
  matlab_addr.sin_port = htons(8080);

    // Create and Bind socket to local address

  client = socket(AF_INET, SOCK_DGRAM, 0);  

    if (client < 0)
    {
       std::cout << "Error creating socket" << std::endl;
       exit(1);
    }

    std::cout << "Socket created" << std::endl;

    if ((bind(client, (struct sockaddr*)&cpp_addr, sizeof(cpp_addr)) ) < 0)
    {
       std::cout << "Error binding connection" << std::endl;
       exit(1);

    }  

    int recvlen;
    int bufSize = 2048;
    unsigned char buf[bufSize];
    socklen_t addrlen = sizeof(matlab_addr);

    //double w = 0.0;

    //char const * message;

    std::cout << "Waiting to receive" << std::endl;

    recvlen = recvfrom(client, buf, bufSize, 0, (struct sockaddr *)&sender_addr, &addrlen);
    //recvlen = recvfrom(client, buf, bufSize, 0, &addrlen);
 
    if (recvlen > 0) {
       buf[recvlen] = 0;
       exit(1);
    }else{
       std::cout << "Nothing received" << std::endl;
       exit(1);
    } 

    double (* bufDouble) = reinterpret_cast<double *>(buf);

   std::cout << "Received is" << bufDouble[0] << " "
                              << bufDouble[1] << " "
                              << bufDouble[2] << " "
                              << bufDouble[3] << " "
                              << bufDouble[4] << " "
                              << bufDouble[5] << std::endl;
*/

   //power at the left human contact 

   double  P_cl = ((jacobian.transpose() * W0_l - Bi * q_dot).transpose()) * q_dot ; // Initial power at left human contact

   // power at the right human contact

   double P_cr = ((jacobian.transpose() * W0_r - Bi * q_dot).transpose()) * q_dot ; // Initial power at right human contact

  // Power of the System 
     
  double P_c = ((jacobian.transpose() * W0 - Bi * q_dot).transpose()) * q_dot ;  // Initial power of the controller
   
  double beta;
  if (P_cl > Pmax){ 
  
        beta = ((((jacobian.transpose() * W0_l).transpose())*q_dot) - Pmax)/ ((q_dot.transpose())*Bi*q_dot);
  }else if(P_cr > Pmax){ 

        beta = ((((jacobian.transpose() * W0_r).transpose())*q_dot) - Pmax)/ ((q_dot.transpose())*Bi*q_dot);
  }else if (P_c > Pmax){ 
  
        beta = ((((jacobian.transpose() * W0).transpose())*q_dot) - Pmax)/ ((q_dot.transpose())*Bi*q_dot);
  }else{
        beta = 1.0; 
  }
  beta_msg.data = beta;               
  beta_pub.publish(beta_msg);


  // New joint damping matrix using scaling parameter beta
  Eigen::MatrixXd B = beta * Bi;

  // New power of the controller using new joint damping matrix
  Eigen::VectorXd tau_cmd(7);
  tau_cmd = (jacobian.transpose()) * W0 - B * q_dot;     // Controller force

  Eigen::VectorXd tau_l(7);
  tau_l = (jacobian.transpose()) * W0_l - B * q_dot;     // Controller force

  Eigen::VectorXd tau_r(7);
  tau_r = (jacobian.transpose()) * W0_r - B * q_dot;     // Controller force

  P_cl = tau_l.transpose() * q_dot ;
  power_msgl.data = P_cl;    
  power_publ.publish(power_msgl);

  P_cr = tau_r.transpose() * q_dot ;
  power_msgr.data = P_cr;    
  power_pubr.publish(power_msgr);

  P_c = tau_cmd.transpose() * q_dot ;      // Power of the controller
  power_msg.data = P_c;    
  power_pub.publish(power_msg);

  std::vector<double> u(7), s(7);      
  Eigen::VectorXd H(7);
  Eigen::VectorXd tauc(7);           

  //state_type integrate_adaptive();
  //integrate_adaptive( rk4() ,my_system, s0 , t0 , t1 , d_t,my_observer);   
  //integrate_adaptive( rkck54() , my_system , s0 , t0 , t1 , d_t, my_observer);   

  //solver to integrate states using integrate_adaptive();
  //integrate_adaptive( make_controlled(1E-12,1E-12,stepper_type()),
  //                    my_system, s0, t0 , t1 , d_t, my_observer ); 
    
  //integrate(my_system, s0, t0 , t1 , d_t);
  
  //ROS_INFO_STREAM("Energy Tanks States : \n" 
  //                << " " << s0[0] << " " << s0[1] << " " << s0[2] << " " << s0[3] << " " << s0[4] << " " << s0[5] << " " << s0[6] << "\n"); 
   
  // adding previous value to the integrated/new state   
  //std::transform(s0.begin(),s0.end(),s0_in.begin(),
  //               s0.begin(),std::plus<double>());
     
  // Controller torques for each joint
  tauc[0] = tau_cmd[0];
  tauc[1] = tau_cmd[1];
  tauc[2] = tau_cmd[2];
  tauc[3] = tau_cmd[3];
  tauc[4] = tau_cmd[4];
  tauc[5] = tau_cmd[5];
  tauc[6] = tau_cmd[6];

  // Energy in each tank, an energy tank is modeled as a spring with const stiffness k = 1
  // connected to robot through a transmission unit 
  // so H = 0.5*k*s^2 ==> H = 0.5*s^2 
  H[0] = 0.5*s0*s0;
  H0_msg.data = H[0];    
  H0_pub.publish(H0_msg);

  H[1] = 0.5*s1*s1;
  H1_msg.data = H[1];    
  H1_pub.publish(H1_msg);

  H[2] = 0.5*s2*s2;
  H2_msg.data = H[2];    
  H2_pub.publish(H2_msg);

  H[3] = 0.5*s3*s3;
  H3_msg.data = H[3];    
  H3_pub.publish(H3_msg);

  H[4] = 0.5*s4*s4;
  H4_msg.data = H[4];    
  H4_pub.publish(H4_msg);

  H[5] = 0.5*s5*s5;
  H5_msg.data = H[5];    
  H5_pub.publish(H5_msg);

  H[6] = 0.5*s6*s6;         
  H6_msg.data = H[6];    
  H6_pub.publish(H6_msg);

  ROS_INFO_STREAM("Energy Tanks Last State: \n" << elapsed_time_ << " " << s_prev0 << " " << s_prev1 << " " << s_prev2 << " " << s_prev3 << " " << s_prev4 << " " << s_prev5 << " " << s_prev6 << "\n");  


  // transmission unit allows power flow from controller to robot and it is regulated by ratio u
  // here u is transmission variable
  if (H[0] > epsilon){    
       u[0] = -tauc[0]/s0;
       s0 = s_prev0 + (-tauc[0]/s_prev0) * q_dot[0] * dt;
  }else{
       u[0] = -tauc[0]/pow(gamma,2)*s0;
       s0 = s_prev0 + ((-tauc[0]/gamma)/gamma)*s_prev0 * q_dot[0] * dt;
  }

  if (H[1] > epsilon){   
       u[1] = -tauc[1]/s1;
       s1 = s_prev1 + (-tauc[1]/s_prev1) * q_dot[1] * dt;
  }else{
       u[1] = -tauc[1]/pow(gamma,2)*s1;
       s1 = s_prev1 + ((-tauc[1]/gamma)/gamma)*s_prev1 * q_dot[1] * dt;
  }

  if (H[2] > epsilon){  
       u[2] = -tauc[2]/s2;
       s2 = s_prev2 + (-tauc[2]/s_prev2) * q_dot[2] * dt; 
  }else{
       u[2] = -tauc[2]/pow(gamma,2)*s2;
       s2 = s_prev2 + ((-tauc[2]/gamma)/gamma)*s_prev2 * q_dot[2] * dt; 
  }

  if (H[3] > epsilon){   
       u[3] = -tauc[3]/s3;
       s3 = s_prev3 + (-tauc[3]/s_prev3) * q_dot[3] * dt; 
  }else{
       u[3] = -tauc[3]/pow(gamma,2)*s3;
       s3 = s_prev3 + ((-tauc[3]/gamma)/gamma)*s_prev3 * q_dot[3] * dt; 
  }

  if (H[4] > epsilon){   
       u[4] = -tauc[4]/s4;
       s4 = s_prev4 + (-tauc[4]/s_prev4) * q_dot[4] * dt; 
  }else{
       u[4] = -tauc[4]/pow(gamma,2)*s4; 
       s4 = s_prev4 + ((-tauc[4]/gamma)/gamma)*s_prev4 * q_dot[4] * dt; 
  }

  if (H[5] > epsilon){  
       u[5] = -tauc[5]/s5;
       s5 = s_prev5 + (-tauc[5]/s_prev5) * q_dot[5] * dt; 
  }else{
       u[5] = -tauc[5]/pow(gamma,2)*s5;
       s5 = s_prev5 + ((-tauc[5]/gamma)/gamma)*s_prev5 * q_dot[5] * dt; 
  }

  if (H[6] > epsilon){ 
       u[6] = -tauc[6]/s6;
       s6 = s_prev6 + (-tauc[6]/s_prev6) * q_dot[6] * dt;
  }else{
       u[6] = -tauc[6]/pow(gamma,2)*s6;
       s6 = s_prev6 + ((-tauc[6]/gamma)/gamma)*s_prev6 * q_dot[6] * dt;
  }

  

  ROS_INFO_STREAM("Energy Tanks States Updated: \n" 
                 << elapsed_time_ << " " << s0 << " " << s1 << " " << s2 << " " << s3 << " " << s4 << " " << s5 << " " << s6 << "\n"); 

  s[0] = s0;
  s[1] = s1;
  s[2] = s2;
  s[3] = s3;
  s[4] = s4;
  s[5] = s5;
  s[6] = s6;

  s_prev0 = s0;
  s_prev1 = s1;
  s_prev2 = s2;
  s_prev3 = s3;
  s_prev4 = s4;
  s_prev5 = s5;
  s_prev6 = s6; 

  //std::vector<double> check_sum{};
  //check_sum.push_back(1.0); 
  //double lets_see = accumulate(check_sum.begin(), check_sum.end(),1);    
  //check_sum.push_back(1);  
  //check_sum.resize(check_sum.size()+1,dt);


  //double t = ros::Time::now().toSec();
  //t += period.toSec();
  //t += period;    // INFINITY    10;
  //double d_t = 0.001; 
  //double t1 = 0.0;
  //t1 = t + d_t;

  //integrate(my_system, s, t , t1 , d_t);
//  ROS_INFO_STREAM("Derivatives : \n" 
//                 << elapsed_time_ << " " << dsdt[0] << " " << dsdt[1] << " " << dsdt[2] << " " << dsdt[3] << " " << dsdt[4] << " " << dsdt[5] << " " << dsdt[6] << "\n");

  
  ROS_INFO_STREAM("Energy for each joint : \n" << elapsed_time_ << " " << H[0] << " " << H[1] << " " << H[2] << " " << H[3] << " " << H[4] << " " << H[5] << " " << H[6] << "\n"); 

  ROS_INFO_STREAM("Energy  : " << " " << E_tot << "\n");

  ROS_INFO_STREAM("Lambda  : " << " " << lamb_da << "\n");

  ROS_INFO_STREAM("Power : " << " " << P_c << "\n");

  ROS_INFO_STREAM("Beta : " << " " << beta << "\n");

//  ROS_INFO_STREAM("Error Position & Orientation: \n" 
//                 << elapsed_time_ << " " << error[0] << " " << error[1] << " " << error[2] << " " << error[3] << " " << error[4] << " " << error[5] << " " << "\n"); 

  ROS_INFO_STREAM("Error : " << " " << et << " " << eo << "\n");
  
  Eigen::VectorXd tau_d(7); 
  //tau_d.setZero(); 
  for (size_t i= 0;  i< 7; ++i) {
       tau_d[i] = -u[i] * s[i];   
       joint_handles_[i].setCommand(tau_d(i));
  }
  ROS_INFO_STREAM("Torques for robot joints : \n"  << " " << tau_d[0] << " " << tau_d[1] << " " << tau_d[2] << " " << tau_d[3] << " " << tau_d[4] << " " << tau_d[5] << " " << tau_d[6] << "\n");  
  
  //ROS_INFO_STREAM("Count : " << " " << count << "\n");
  //++count;

}
}


}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::EnergyShapingControllerToFile,
                       controller_interface::ControllerBase)
