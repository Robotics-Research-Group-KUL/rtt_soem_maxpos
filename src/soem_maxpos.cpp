/***************************************************************************
 soem_maxpos.cpp -  description
 -------------------
 begin                : October 8 2019
 copyright            : (C) 2019 Gianni Borghesan, Sergio Portolez
 email                : first.last@kuleuven.be

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include "soem_maxpos.h"
#include <soem_master/soem_driver_factory.h>

#include "math.h"

using namespace RTT;

/*
Command     LowByte of Controlword[binary] State Transition
Shutdown    0xxx x110     2, 6, 8
Switch On   0xxx x111     3
Switch On & Enable Operation    0xxx 1111     3, 4 *1)
Disable Voltage     0xxx xx0x     7, 9, 10, 12
Quick Stop    0xxx x01x    7, 10, 11
Disable Operation     0xxx 0111     5
Enable Operation    0xxx 1111    4, 16
Fault Reset     0xxx xxxx -> 1xxx xxxx    15
*/

void SoemMaxPos::cw_reset(){//0xxx x110
  new_control_word.set(7,false);
  new_control_word.set(0,false);
  new_control_word.set(1);
  new_control_word.set(2);
}
void SoemMaxPos::cw_switch_on(){//0xxx x111
  new_control_word.set(7,false);
  new_control_word.set(0);
  new_control_word.set(1);
  new_control_word.set(2);
}
void SoemMaxPos::cw_switch_on_and_enable_operation(){//0xxx 1111
  new_control_word.set(7,false);
  new_control_word.set(0);
  new_control_word.set(1);
  new_control_word.set(2);
  new_control_word.set(3);
}
void SoemMaxPos::cw_disable_voltage(){// 0xxx xx0x
  new_control_word.set(7,false);
  new_control_word.set(1,false);
}
void SoemMaxPos::cw_quick_stop(){// 0xxx x01x
  new_control_word.set(7,false);
  new_control_word.set(2,false);
  new_control_word.set(1);
}
void SoemMaxPos::cw_disable_operation(){// 0xxx 0111
  new_control_word.set(7,false);
  new_control_word.set(3,false);
  new_control_word.set(0);
  new_control_word.set(1);
  new_control_word.set(2);
}
void SoemMaxPos::cw_enable_operation(){// 0xxx 1111
  new_control_word.set(7,false);
  new_control_word.set(0);
  new_control_word.set(1);
  new_control_word.set(2);
  new_control_word.set(3);
}
void SoemMaxPos::cw_fault_reset(){// 1xxx xxxx (should be 0xxx xxxx before)
  new_control_word.set(7);
}
bool  SoemMaxPos::velocity_ramp(double velocity,double accelleration){
	if(false==_set_mode_of_operation(3)) {
		return false;
	}
	int speed=(int)velocity;
	uint accel=(uint)accelleration;
	int retval;

	retval = ec_SDOwrite(m_slave_nr, 0x60FF, 0x00, FALSE, sizeof(speed), &speed, EC_TIMEOUTSAFE);
	retval = ec_SDOwrite(m_slave_nr, 0x6083, 0x00, FALSE, sizeof(accel), &accel, EC_TIMEOUTSAFE);
	retval = ec_SDOwrite(m_slave_nr, 0x6084, 0x00, FALSE, sizeof(accel), &accel, EC_TIMEOUTSAFE);
return true;
}
/*
 * Target Velocity 0x60FF The speed the drive is supposed to reach
Profile Acceleration 0x6083 Defines the acceleration ramp during a movement
Profile Deceleration 0x6084*/


SoemMaxPos::SoemMaxPos(ec_slavet* mem_loc) :
  soem_master::SoemDriver(mem_loc)
, downsample(10)
, encoder_thick_per_revolution(512)
, gear_ratio(1.0)
, motor_rated_torque(0.0)

{
  m_service->doc(std::string("MaxPos") + std::string(
                   m_datap->name) );

  //output ports
  m_service->addPort("status",status_word_outport).doc("written if values changes,string with current state of the MAXPOS");
  m_service->addPort("status_8MSB",status_word_mode_dep_outport).doc("written if values changes, state with the 8MSB of the statusword, mode dependent meaning");
  m_service->addPort("digital_inputs",digital_inputs_outport).doc("written if values changes");
  m_service->addPort("touch_probe",touch_probe_status_outport).doc("written if values changes");

  m_service->addPort("modes_of_operation_display",modes_of_operation_display_outport)
		  .doc("current mode of operation");

  m_service->addPort("position",position_outport);
  m_service->addPort("position_ros",position_outport_ds).doc("downsampled port");
  m_service->addPort("velocity",velocity_outport);
  m_service->addPort("velocity_ros",velocity_outport_ds).doc("downsampled port");
  m_service->addPort("torque",torque_outport);
  m_service->addPort("torque_ros",torque_outport_ds).doc("downsampled port");

  //input ports
  m_service->addPort("target_position",target_position_inport);
  m_service->addPort("target_velocity",target_velocity_inport);
  m_service->addPort("target_torque",target_torque_inport);

  m_service->addProperty("encoder_thick_per_revolution",encoder_thick_per_revolution);
  m_service->addProperty("gear_ratio",gear_ratio);

  m_service->addProperty("downsample",downsample).doc("Downsaple ratio for ds (pos/vel/torque) ports, if 0 will disable it");

  //operations for escaling the state machine
  m_service->addOperation("reset", &SoemMaxPos::cw_reset, this, RTT::OwnThread).doc(
        "First command to be issued");
  m_service->addOperation("switch_on", &SoemMaxPos::cw_switch_on, this, RTT::OwnThread).doc(
        "Second command to be issued");
  m_service->addOperation("enable_operation", &SoemMaxPos::cw_enable_operation, this, RTT::OwnThread).doc(
        "Third command to be issued");
  m_service->addOperation("switch_on_and_enable_operation", &SoemMaxPos::cw_switch_on_and_enable_operation, this, RTT::OwnThread).doc(
        "two commands together");
  m_service->addOperation("disable_voltage", &SoemMaxPos::cw_disable_voltage, this, RTT::OwnThread).doc(
        "check manual");
  m_service->addOperation("disable_operation", &SoemMaxPos::cw_disable_operation, this, RTT::OwnThread).doc(
        "check manual");
  m_service->addOperation("quick_stop", &SoemMaxPos::cw_quick_stop, this, RTT::OwnThread).doc(
        "check manual");
  m_service->addOperation("fault_reset", &SoemMaxPos::cw_fault_reset, this, RTT::OwnThread).doc(
        "check manual");


  m_service->addOperation("set_mode_of_operation", &SoemMaxPos::set_mode_of_operation, this, RTT::OwnThread).doc(
			"change operational mode, returns true on success").arg("desired mode of operation"," values\n"
																								"/t/t1 Profile Position Mode (PPM) -not implemented \n"
	        		                                                                                                        "/t/t3 Profile Velocity Mode (PVM) -not implemented \n"
	                                                                                            "/t/t6 Homing Mode (HMM) -not implemented n"
	                                                                                            "/t/t8 Cyclic Synchronous Position Mode (CSP)\n"
	                                                                                            "/t/t9 Cyclic Synchronous Velocity Mode (CSV)\n"
	                                                                                            "/t/t10 Cyclic Synchronous Torque Mode (CST)");
  m_service->addOperation("velocity_ramp", &SoemMaxPos::velocity_ramp, this, RTT::OwnThread);
  status_word_msg.data="NOT_READY_TO_SWITCH_ON"; //set longest string
  status_word_outport.setDataSample(status_word_msg);
}

bool SoemMaxPos::_set_mode_of_operation(int mode)
{
	  int8 value=mode;
	  int8 new_value=0;
	  int n_of_bytes=sizeof(new_value);
	  int retval;

	  retval = ec_SDOwrite(m_slave_nr, 0x6060, 0x00, FALSE, sizeof(value), &value, EC_TIMEOUTSAFE);
	  retval = ec_SDOread(m_slave_nr,  0x6061, 0x00, FALSE, &n_of_bytes , &new_value, EC_TIMEOUTSAFE);

	  if (value!=new_value){
	      Logger::In in(this->getName());
	      log(Error)<< m_datap->name<<" : not able to set requested mode of operation.\n" << endlog();
	      return false;
	    }
	  current_mode_of_operation=value;
	  return true;
}

bool SoemMaxPos::set_mode_of_operation(int mode)
{
  if (mode!=8 && mode!=9 && mode!=10){
      Logger::In in(this->getName());
      log(Error)<< m_datap->name<<" : requested mode is not implemented,"
    		  " only 8,9,10 currently implemented." << endlog();
      return false;
    }
 return _set_mode_of_operation(mode);
}

bool SoemMaxPos::configure()
{	 
  // Get period of the owner of the service, i.e. the soem master
  //this is not really needed in there are no trajectory inside the component
  //RTT::TaskContext *upla = m_service->getOwner();
  //std::cout << m_name << ": Name of owner: " << upla->getName() << std::endl;
  //ts_ = upla->getPeriod();
  //std::cout << m_name << ": Period of owner: " << ts_ << std::endl;
  // Set initial configuration parameters
  //we need to read the motor rated torque in order to be able convert % to phisical value
  //motor_rated_torque_unit is in MICRO Nm -> motor_rated_torque is in Nm
  uint32 motor_rated_torque_unit;
  int n_of_bytes=sizeof(motor_rated_torque_unit), retval;
  retval = ec_SDOread(m_slave_nr,  0x6076, 0x00, FALSE, &n_of_bytes , &motor_rated_torque_unit, EC_TIMEOUTSAFE);
  if (0==retval || sizeof(motor_rated_torque_unit)!=n_of_bytes){
      Logger::In in(this->getName());
      log(Error)<< m_datap->name<<" : Problem reading motor_rated_torque value from slave. Values: retval: "<<retval
                <<" n_of_bytes: "<<n_of_bytes<<" sizeof(motor_rated_torque_unit): "<<sizeof(motor_rated_torque_unit)  << endlog();
      return false;
    }
  motor_rated_torque=((double)motor_rated_torque_unit)/1000000.0;
  m_service->addConstant("motor_rated_torque",motor_rated_torque);

  return true;
  iteration=0;
}
bool SoemMaxPos::start(){
  ((control_msg*) (m_datap->outputs))->target_position=(int32) 0;
  ((control_msg*) (m_datap->outputs))->position_offset=(int32) 0;
  ((control_msg*) (m_datap->outputs))->velocity_offset=(int32) 0;
  ((control_msg*) (m_datap->outputs))->torque_offset=(int16) 0;
  return true;
}


void SoemMaxPos::update()
{
  std_msgs::UInt32 digital_inputs;
  std_msgs::UInt16 touch_probe_status;
  std_msgs::UInt8 modes_of_operation_display;

  double pos_coversion_factor=((2*M_PI)/encoder_thick_per_revolution)/gear_ratio; //thick->radiants/sec (or other depending by the unit of the gear ratio)
  double vel_coversion_factor=((2*M_PI)/60.0)/gear_ratio;
  double tau_coversion_factor=(motor_rated_torque/1000.0)*gear_ratio;
  // ****************************
  // *** Read data from slave ***
  // ****************************

  double position= (double)((read_mgs*) (m_datap->inputs))->position_actual_value;//rev
  double velocity= (double)((read_mgs*) (m_datap->inputs))->velocity_actual_value;//rpm
  double torque= (double)((read_mgs*) (m_datap->inputs))->torque_actual_value;//percentage of motor_rated_torque

  position*=pos_coversion_factor;
  velocity*=vel_coversion_factor;
  torque*=tau_coversion_factor;

  uint16 status_word_uint= ((read_mgs*) (m_datap->inputs))->status_word;
  std::bitset<16> status_word(status_word_uint);
  touch_probe_status.data= ((read_mgs*) (m_datap->inputs))->touch_probe_status;
  digital_inputs.data= ((read_mgs*) (m_datap->inputs))->digital_inputs;

  modes_of_operation_display.data =((read_mgs*) (m_datap->inputs))->modes_of_operation_display;

  //TODO check for error in the status word

  //writes on port only if new values are received

  if  (status_word!=last_status_word){//change of state
      //check least significative 8 bit
      uint8 sw_ls=static_cast<uint8>(status_word.to_ulong());
      uint8 lsw_ls=static_cast<uint8>(last_status_word.to_ulong());
      if (sw_ls!=lsw_ls)
        {
          if (!state_to_string(status_word,status_word_msg.data))
            status_word_msg.data="ERROR_IN_STATE_INTERPRETATION";
          status_word_outport.write(status_word_msg);
        }
      //check most significative 8 bit
      uint8 sw_ms=static_cast<uint8>(status_word.to_ulong()>>8);
      uint8 lsw_ms=static_cast<uint8>(last_status_word.to_ulong()>>8);
      if (sw_ms!=lsw_ms)
        {
          std_msgs::UInt8 status_word_high;
          status_word_high.data  =sw_ms;


          status_word_mode_dep_outport.write(status_word_high);
        }
      last_status_word=status_word;
    }
  if  (digital_inputs.data!=last_digital_inputs.data){
      digital_inputs_outport.write(digital_inputs);
      last_digital_inputs=digital_inputs;
    }
  if  (touch_probe_status.data!=last_touch_probe_status.data){
      touch_probe_status_outport.write(touch_probe_status);
      last_touch_probe_status=touch_probe_status;
    }

  position_outport.write(position);
  velocity_outport.write(velocity);
  torque_outport.write(torque);
  modes_of_operation_display_outport.write(modes_of_operation_display);
  if (downsample!=0){
      if(iteration>=downsample-1){
          std_msgs::Float32 d;
          d.data=position;
          position_outport_ds.write(d);
          d.data=velocity;
          velocity_outport_ds.write(d);
          d.data=torque;
          torque_outport_ds.write(d);
          iteration=0;
        }
      else iteration++;
    }

  // ***************************
  // *** Write data to slave ***
  // ***************************



  double data;
  if (target_position_inport.read(data)!=RTT::NoData){
      ((control_msg*) (m_datap->outputs))->target_position = (int32)(data/pos_coversion_factor);
    }
  if (target_velocity_inport.read(data)!=RTT::NoData){

	  // ((control_msg*) (m_datap->outputs))->velocity_offset = (int32)(data/vel_coversion_factor);

	  int retval ;
	  int vel=(int32)(data/vel_coversion_factor);
	  //retval= ec_SDOwrite(m_slave_nr, 0x60FF, 0x00, FALSE, sizeof(vel), &vel, EC_TIMEOUTSAFE);
	  /*std::cout<<"retval: "<<retval<<std::endl;
	  int demanded_vel;
      int n_of_bytes=sizeof(demanded_vel);
      demanded_vel=-1;
      retval = ec_SDOread(m_slave_nr,  0x606B, 0x00, FALSE, &n_of_bytes , &demanded_vel, EC_TIMEOUTSAFE);
      std::cout<<"demanded vel: "<<demanded_vel<<std::endl;
      uint max_speed=-1; n_of_bytes=sizeof(max_speed);
      retval = ec_SDOread(m_slave_nr,  0x6080, 0x00, FALSE, &n_of_bytes , &max_speed, EC_TIMEOUTSAFE);
            std::cout<<"max_speed: "<<max_speed<<std::endl;*/

    }
  if (target_torque_inport.read(data)!=RTT::NoData){
      ((control_msg*) (m_datap->outputs))->torque_offset = (int16)(data/tau_coversion_factor);
    }
  //control word is set via operation
  ((control_msg*) (m_datap->outputs))->control_word = (unsigned short)new_control_word.to_ulong();
  ((control_msg*) (m_datap->outputs))->modes_of_operation = current_mode_of_operation;

}


namespace
{
  soem_master::SoemDriver* createSoemMaxPos(ec_slavet* mem_loc)
  {
    return new SoemMaxPos(mem_loc);
  }
  const bool registered0 =
      soem_master::SoemDriverFactory::Instance().registerDriver("MAXPOS",
                                                                createSoemMaxPos);

}

