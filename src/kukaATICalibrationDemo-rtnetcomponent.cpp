// Filename: kukaATICalibrationDemo-rtnetcomponent.cpp
// Copyright: 2014 ISIR-CNRS
// Author: Sovan Hak, Guillaume Hamon (hak@isir.upmc.fr, hamon@isir.upmc.fr)
// Description:

#include "kukaATICalibrationDemo-rtnetcomponent.hpp"
#include <rtt/Component.hpp>
#include <iostream>


#include <boost/foreach.hpp>
#include <math.h>

KukaATICalibrationDemoRTNET::KukaATICalibrationDemoRTNET(std::string const& name) : FriRTNetExampleAbstract(name){
	this->addPort("ATI_i", iport_ATI_values); // gets ATI F/T sensor values
  	this->addPort("ATI_calibration_results", oport_calibration_results); // send the measurement to process sensor calibration
  	this->addPort("BiasOrder_o",oport_bias_order); // Ask ATISensor component to order a bias command to the sensor
	this->addPort("addJntTorque_o",oport_add_joint_torque); // used to perform load compensation
	this->addPort("estExtJntTrq_i",iport_est_ext_joint_torque); // gets the external force/load estimation
	this->addOperation("setJointImpedance", &KukaJacobianDemoRTNET::setJointImpedance, this, RTT::OwnThread);
	dT=this->getPeriod();

	velocity_limit=0.2;//0.2; //T1: 250mm/s max along the end effector, arbitrary value of 0.2 rad/s at the joints
  	end_calibration=false;
  	t=0;
	n=0;
  	valeurZ.resize(6); // Fx,Fy,Fz,Tx,Ty,Tz
  	valeurX.resize(6);
  	valeurY.resize(6);
  	tf_min.resize(LWRDOF);
  	position1.resize(LWRDOF);
  	position2.resize(LWRDOF);
  	position3.resize(LWRDOF);
	position4.resize(LWRDOF);
  	joint_position_command.resize(LWRDOF);
  	joint_position_command_interp.resize(LWRDOF);
	external_torque.resize(LWRDOF);
}


bool KukaATICalibrationDemoRTNET::doStart(){
    //setting stiffness
	std::vector<double> stiff(LWRDOF, 250.0);
	std::vector<double> damp(LWRDOF, 0.1);
	setJointImpedance(stiff, damp);

 	RTT::FlowStatus joint_state_fs=iport_msr_joint_pos.read(JState_init);
	for(i=0;i<7;i++){
  		tf_min[i]=(15*std::abs(joint_position_command[i]-JState[i])/(8*velocity_limit));
  	}
  	tf=tf_min[0];
  	for(i=1;i<7;i++){
		if(tf_min[i]>tf){
			tf=tf_min[i];
		}
  	}
	RTT::FlowStatus estExtTrq_fs=iport_est_ext_joint_torque.read(external_torque);
	if (estExtTrq_fs==RTT::NewData){
		for( i=0;i<7;i++ ){
         		external_torque[i]=external_torque[i];
		}
		oport_add_joint_torque.write(external_torque);
	}

     	friStart();
     	return true;
}

bool KukaATICalibrationDemoRTNET::configureHook(){
	//vectors initializations
	double w[7]={0,0,0,1.57,0,-1.57,0};
	position1.assign(&w[0],&w[0]+7);
	w[5]=0;
	position2.assign(&w[0],&w[0]+7);
	w[6]=1.57;
	position3.assign(&w[0],&w[0]+7);

	for(i=0;i<7;i++)
	{
		position4[i]=0;
         	external_torque[i] = 0;
  	}

  	joint_position_command = position1;
    	setPeer("lwr");
    	//initialize the arrays that will be send to KRL
    	for(int i=0; i<16; ++i){
        	fri_to_krl.intData[i]=0;
        	fri_to_krl.realData[i]=0.0;
   	}
    	return true;
}


void KukaATICalibrationDemoRTNET::updateHook(){

    std::string fri_mode("e_fri_unkown_mode");
    bool fri_cmd_mode = false;
    RTT::FlowStatus joint_state_fs = iport_msr_joint_pos.read(JState);
    RTT::FlowStatus fs_event = iport_events.read(fri_mode);
    if (fri_mode == "e_fri_cmd_mode")
        fri_cmd_mode = true;
    else if (fri_mode == "e_fri_mon_mode")
        fri_cmd_mode = false;


	if(fri_cmd_mode){
		if(joint_state_fs == RTT::NewData){
			if(!end_calibration){
				if(joints_position_command == position1 && t==tf){ // first position reached, saving sensor values
					iport_ATI_values.read(valeurZ);
					oport_bias_order.write(true);
					// changing command to position 2
					joint_position_command = position2;
					JState_init=JState;
					t=0;
					for(i=0;i<7;i++){
						tf_min[i]=(15*std::abs(joints_position_command[i]-JState_init[i])/(8*velocity_limit));
					}
					tf=tf_min[0];
					for(i=1;i<7;i++){
						if(tf_min[i]>tf){
							tf=tf_min[i];
						}
  					}

  				}else{
					if(joints_position_command == position2 && t == tf){// second position reached, saving sensor values
         					iport_ATI_values.read(valeurX);
		         			// changing command to position 3
         					joint_position_command = position3;
						JState_init=JState;
						t=0;
						for(i=0;i<7;i++){
							tf_min[i]=(15*std::abs(joints_position_command[i]-JState_init[i])/(8*velocity_limit));
						}
						tf=tf_min[0];
						for(i=1;i<7;i++){
							if(tf_min[i]>tf){
								tf=tf_min[i];
							}
  						}

  					}else{
						if(joints_position_command == position3 && t==tf){// third position reached, saving sensor values
                 					iport_ATI_values.read(valeurY);
                 					// end of calibration
							JState_init=JState;
							end_calibration = true;
         					}
					}
  				}
  			}else{ // calibration ended
				if(joints_position_command != position4){
				// calibration ended, send sensor measurements to ATISensor component which will process sensor's weight compensation computations
					Eigen::MatrixXd results(3,6);
					results.row(0)=Eigen::VectorXd::Map(&valeurZ[0],valeurZ.size());
					results.row(1)=Eigen::VectorXd::Map(&valeurX[0],valeurX.size());
					results.row(2)=Eigen::VectorXd::Map(&valeurY[0],valeurY.size());
					oport_calibration_results.write(results);

					/*goes back to home position, verifying that Fnorm value is close to zero (active weight compensation) */
					joints_position_command = position4;
					JState_init=JState;
					t=0;
					for(i=0;i<7;i++){
						tf_min[i]=(15*std::abs(joints_position_command[i]-JState_init[i])/(8*velocity_limit));
					}
					tf=tf_min[0];
					for(i=1;i<7;i++){
						if(tf_min[i]>tf){
							tf=tf_min[i];
						}
  					}
				}
				if (t==tf){
					// home position reached, ending component life
					FriRTNetExampleAbstract::stop();
					//return;
				}
  			}
  			// polynomoiale interpolation of 5th degree to perform continuous position, velocity and acceleration
  			for(i=0;i<7;i++){
  				joint_position_command_interp[i]=JState_init[i]+(joint_position_command[i]-JState_init[i])*(10*pow(t/tf,3)-15*pow(t/tf,4)+6*pow(t/tf,5));
  			}

			if(requiresControlMode(30)){
				if (estExtTrq_fs==RTT::NewData){
					for(i=0; i<7; i++ ){
         					external_torque[i]=external_torque[i];
    					}
				oport_add_joint_torque.write(external_torque);
				}
			}
			oport_joint_position.write(joint_position_command_interp);

  			t+=dT;
  			if(t>tf){
				t=tf;
  			}
		}else{
			std::cout<<"Cannot read Joint position Port"<<std::endl;
		}

  	}
}


void KukaATICalibrationDemoRTNET::setJointImpedance(std::vector<double> &stiffness, std::vector<double> &damping){
	if(stiffness.size() != LWRDOF || damping.size() != LWRDOF){
		std::cout << "Wrong vector size, should be " << LWRDOF << ", " << LWRDOF << std::endl;
		return;
	}else{
		lwr_fri::FriJointImpedance joint_impedance_command;
		for(unsigned int i = 0; i < LWRDOF; i++){
			joint_impedance_command.stiffness[i] = stiffness[i];
			joint_impedance_command.damping[i] = damping[i];
		}

		oport_joint_impedance.write(joint_impedance_command);
	}
}

/*
* Using this macro, only one component may live
* in one library *and* you may *not* link this library
* with another component library. Use
* ORO_CREATE_COMPONENT_TYPE()
* ORO_LIST_COMPONENT_TYPE(kuka_jacobian_demo)
* In case you want to link with another library that
* already contains components.
*
* If you have put your component class
* in a namespace, don't forget to add it here too:
*/
ORO_CREATE_COMPONENT(KukaATICalibrationDemoRTNET)
