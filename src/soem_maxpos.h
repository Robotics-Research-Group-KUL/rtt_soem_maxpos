

#ifndef SOEM_MAXPOS_H
#define SOEM_MAXPOS_H

#include <soem_master/soem_driver.h>
#include <bitset>
#include <rtt/Port.hpp>
#include <bitset>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
/*
  [0x0000.0] 0x6040:0x00 0x10 UNSIGNED16   Controlword
  [0x0002.0] 0x607A:0x00 0x20 INTEGER32    Target Position
  [0x0006.0] 0x60FF:0x00 0x20 INTEGER32    Target Velocity
  [0x000A.0] 0x6071:0x00 0x10 INTEGER16    Target Torque
  [0x000C.0] 0x60FE:0x01 0x20 UNSIGNED32   Physical Outputs
  [0x0010.0] 0x60B8:0x00 0x10 UNSIGNED16   Touch Probe Function
  [0x0012.0] 0x60B1:0x00 0x20 INTEGER32    Velocity Offset
  SM3 inputs
     addr b   index: sub bitl data_type    name
  [0x0016.0] 0x6041:0x00 0x10 UNSIGNED16   Statusword
  [0x0018.0] 0x6064:0x00 0x20 INTEGER32    Position Actual Value
  [0x001C.0] 0x606C:0x00 0x20 INTEGER32    Velocity Actual Value
  [0x0020.0] 0x6077:0x00 0x10 INTEGER16    Torque Actual Value
  [0x0022.0] 0x60FD:0x00 0x20 UNSIGNED32   Digital Inputs
  [0x0026.0] 0x60B9:0x00 0x10 UNSIGNED16   Touch Probe Status

*/
//modes_of_operation
#define MODE_OP_PROFILE_POS 1
#define MODE_OP_PROFILE_VEL 3
#define MODE_OP_HOMING 6
#define MODE_OP_CYCLIC_SYNCHRONOUS_POS 8
#define MODE_OP_CYCLIC_SYNCHRONOUS_VEL 9
#define MODE_OP_CYCLIC_SYNCHRONOUS_TORQUE 10

//State of the Drive - table 3.5 since values are on 7 bits, the 8th bit (most siginificant is set to zero)
#define NOT_READY_TO_SWITCH_ON  0b00000000
#define SWITCH_ON_DISABLED      0b01000000
#define READY_TO_SWITCH_ON      0b00100001
#define SWITCHED_ON             0b00100011
#define OPERATION_ENABLE        0b00110111
#define QUICK_STOP_ACTIVE       0b00010111
#define FAULT_REACTION_ACTIVE   0b00011111
#define FAULT                   0b00001000
//note: fault is the 4th bit set

class SoemMaxPos : public soem_master::SoemDriver
{

  typedef struct PACKED
  {
    uint16 control_word;
    int32 target_position;
    int32 target_velocity;
    int16 target_Torque;
    uint32 physical_outputs;
    uint16 touch_probe_function;
    int32 velocity_offset;
  } control_msg;

  typedef struct PACKED
  {
    uint16 status_word;
    int32  position_actual_value;
    int32  velocity_actual_value;
    int16  torque_actual_value;
    uint32  digital_inputs;
    uint16 touch_probe_status;
  } read_mgs;

public:
  SoemMaxPos(ec_slavet* mem_loc);
  ~SoemMaxPos(){}

  bool configure();
  void update();
  //functioons to set the command word, needed to escale the state machine and ring the module in working state
  void cw_reset();
  void cw_switch_on();
  void cw_switch_on_and_enable_operation();
  void cw_disable_voltage();
  void cw_quick_stop();
  void cw_disable_operation();
  void cw_enable_operation();
  void cw_fault_reset();

  bool set_mode_of_operation(int mode);
private:
  //RTT::OutputPort<std::vector<double> > position_outport;
  int ros_downsample;
  RTT::OutputPort<std_msgs::String > status_word_outport;
  RTT::OutputPort<std_msgs::Float32 > position_outport_ds;
  RTT::OutputPort<double > position_outport;
  RTT::OutputPort<std_msgs::Float32 > velocity_outport_ds;
  RTT::OutputPort<double > velocity_outport;
  RTT::OutputPort<std_msgs::Float32 > torque_outport_ds;
  RTT::OutputPort<double > torque_outport;
  RTT::OutputPort<std_msgs::UInt32 > digital_inputs_outport;
  RTT::OutputPort<std_msgs::UInt16 > touch_probe_status_outport;

  RTT::InputPort<double> target_pos;
  RTT::InputPort<double> target_vel;
  RTT::InputPort<double> target_torque;



  std_msgs::String status_word_msg;
  std::bitset<16> new_control_word;
  std::bitset<16> last_status_word;
  std_msgs::UInt32 last_digital_inputs;
  std_msgs::UInt16 last_touch_probe_status;


  int iteration;
  //properties
  double rev_position_ratio;
  double rev_velocity_ratio;

  int downsample;

};

#endif
