// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_example_controllers {

class CartesianTorqueController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  //ros::Time current_time;
  ros::Duration elapsed_time_;
  std::array<double, 16> initial_pose_{};
  ros::Duration t_vec;
  //double plot_time;

  //int client;   //, recvlen, bufSize;
  //unsigned char buf[2048];
  //socklen_t addrlen, sender_addr;

  //std::array<double, 7> s;
  //std::array<double, 7> dsdt;
  //std::array<double, 7> ds;
  //std::array<double, 7> s_prev;
  
  //ros::Subscriber sub;
  ros::Publisher tvec_pub;
  //ros::Publisher energy_pub;
  //ros::Publisher lambda_pub;
  //ros::Publisher power_publ;
  //ros::Publisher power_pubr;
  //ros::Publisher power_pub;
  //ros::Publisher beta_pub;
  ros::Publisher et_pub;
  ros::Publisher eo_pub;

  ros::Publisher x_position_pub;
  ros::Publisher y_position_pub;
  ros::Publisher z_position_pub;

  ros::Publisher x_position_d_pub;
  ros::Publisher y_position_d_pub;
  ros::Publisher z_position_d_pub;

  //std::vector<double> ftleft(7);
  //std::vector<double> time_vec{};
  //std::vector<double> stiff_vec{};

  //std::vector<double> ds0, ds1, ds2, ds3, ds4, ds5, ds6;
/*
  std::vector<double> ds0;
  std::vector<double> ds1;
  std::vector<double> ds2;
  std::vector<double> ds3;
  std::vector<double> ds4;
  std::vector<double> ds5;
  std::vector<double> ds6;
*/
  //std::vector<double> u, q_share;     
  //Eigen::Matrix<double, 7, 1> H;
  //Eigen::Matrix<double, 7, 1> tauc;           

  //ros::Duration t;
  //double elapsed_time_;
  //double t;
  
  //std::array<double, 7> s;
  //typedef std::vector<double> state_type;

  //Eigen::Matrix<double, 7, 1> tau_J_d;  
  //Eigen::Matrix<double, 7, 1> q_dot;
  //Eigen::Matrix<double, 7, 1> u;

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;


  // Dynamic reconfigure
//  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
//      dynamic_server_compliance_param_;
//  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;

  //void my_system(const std::vector<double> &s, std::vector<double> &sdot, const ros::Time& time);
  //void my_observer(const std::vector<double> &s , const double t) ; 
  //std::vector<double> udp(std::vector<double> &ftleft);
  //void udp();
};

}  // namespace franka_example_controllers
