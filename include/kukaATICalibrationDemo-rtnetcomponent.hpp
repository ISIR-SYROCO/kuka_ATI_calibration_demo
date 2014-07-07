// Copyright (C) 2014 ISIR-CNRS
// Author: Guillaume Hamon, hamon@isir.upmc.fr

#ifndef OROCOS_KUKAATICALIBRATIONDEMORTNET_COMPONENT_HPP
#define OROCOS_KUKAATICALIBRATIONDEMORTNET_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include "friRTNetExampleAbstract.hpp"
#include <Eigen/Dense>
#include <rtt/Component.hpp>
#include <iostream>
#include <cmath>
#include <boost/foreach.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/typekit/Types.h>

class KukaATICalibrationDemoRTNET : public FriRTNetExampleAbstract{
  public:
    KukaATICalibrationDemoRTNET(std::string const& name);
    bool configureHook();
    bool doStart();
    void updateHook();
    void setJointImpedance(std::vector<double> &stiffness, std::vector<double> &damping);

    std::vector<double> JState_init;//msr_joint_positions
    bool end_calibration;
    std::vector<double> joint_position_command;
    std::vector<double> joint_position_command_interp;
   //Positions de calibration
    std::vector<double> position1;
    std::vector<double> position2;
    std::vector<double> position3;
    std::vector<double> position4;
    std::vector<double> JState;

    double velocity_limit;
    double tf;
    double t;
    int n;
    float dT;

    std::vector<double> valeurZ;
    std::vector<double> valeurX;
    std::vector<double> valeurY;

    std::vector<double> tf_min;

    std::vector<double> external_torque;
    std::vector<double> msr_torque;

    int i;
    RTT::InputPort< std::vector<double> > iport_msr_joint_torque;
    RTT::InputPort< std::vector<double> > iport_ATI_values;
    RTT::InputPort< std::vector<double> > iport_est_ext_joint_torque;
    RTT::OutputPort< Eigen::Matrix<double,3,6> > oport_calibration_results;
    RTT::OutputPort< bool > oport_bias_order;
    RTT::OutputPort< std::vector<double> > oport_add_joint_torque;

};
#endif
