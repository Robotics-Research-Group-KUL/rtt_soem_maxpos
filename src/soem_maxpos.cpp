/***************************************************************************
 tag: Bert Willaert  Fri Sept 21 09:31:20 CET 2012  soem_robotiq_3Finger.cpp

 soem_robotiq_3Finger.cpp -  description
 -------------------
 begin                : Fri September 21 2012
 copyright            : (C) 2012 Bert Willaert
 email                : first.last@mech.kuleuven.be

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

using namespace RTT;


SoemMaxPos::SoemMaxPos(ec_slavet* mem_loc) :
    soem_master::SoemDriver(mem_loc)
{
    m_service->doc(std::string("MaxPos") + std::string(
                       m_datap->name) );


}

bool SoemMaxPos::configure()
{	 
    // Get period of the owner of the service, i.e. the soem master
    /*RTT::TaskContext *upla = m_service->getOwner();
    std::cout << m_name << ": Name of owner: " << upla->getName() << std::endl;
    ts_ = upla->getPeriod();
        std::cout << m_name << ": Period of owner: " << ts_ << std::endl;
    // Set initial configuration parameters
    SetMode(0,0);
    SetOptions(0,0);
    SetVelocity(0.8);
        SetForce(0.5);*/
    return true;
}



void SoemMaxPos::update()
{


    // ******************************
    // *** Read data from gripper ***
    // ******************************
    uint16 status_word= ((in_1a00*) (m_datap->inputs))->status_word;
    int32  position_actual_value= ((in_1a00*) (m_datap->inputs))->position_actual_value;

        std::cout<<position_actual_value<<std::endl;
    /*	 // gACT
    status_msg.ACT = (int32) ( (  (input_bits.to_ulong()) & BYTE0_ACT ) >> LSB_NonZeroBit(BYTE0_ACT));
     // Mode
     status_msg.MOD = (int32) ( (  (input_bits.to_ulong()) & BYTE0_MOD ) >> LSB_NonZeroBit(BYTE0_MOD));
     // gGTO
    status_msg.GTO = (int32) ( (  (input_bits.to_ulong()) & BYTE0_GTO ) >> LSB_NonZeroBit(BYTE0_GTO));
     // IMC
    status_msg.IMC = (int32) ( (  (input_bits.to_ulong()) & BYTE0_IMC ) >> LSB_NonZeroBit(BYTE0_IMC));
     // STA
    status_msg.STA = (int32) ( (  (input_bits.to_ulong()) & BYTE0_STA ) >> LSB_NonZeroBit(BYTE0_STA));

     // ******************************
     // *** Read data from gripper ***
     // ******************************
    fault_bits = ((in_Robotiqt*) (m_datap->inputs))->FaultStatus;
     for (int i = 0; i<4; i++)
            status_msg.Fault[i] = fault_bits[i];

     // *****************************************
     // *** Read data from individual fingers ***
     // *****************************************
     // Note that the scissor dof is also considered a finger //
    input_bits = ((in_Robotiqt*) (m_datap->inputs))->ObjectDetection;
     for (int i = 0; i<4; i++){
         // DTi
         int BYTE1_DTi = ((0x01 | 0x02) << (i*2));
       status_msg.Fingers[i].DTi = (int32) ( (  (input_bits.to_ulong()) & BYTE1_DTi ) >> LSB_NonZeroBit(BYTE1_DTi));
         // Requested Position
         status_msg.Fingers[i].ReqPosition = ((in_Robotiqt*) (m_datap->inputs))->Fingers[i].FingerPosRequestEcho;
         // Actual Position
         status_msg.Fingers[i].ActPosition = ((in_Robotiqt*) (m_datap->inputs))->Fingers[i].FingerPosition;
         // Actual Current
         status_msg.Fingers[i].Current = ((in_Robotiqt*) (m_datap->inputs))->Fingers[i].FingerCurrent;
     }

     status_port.write(status_msg);

     // *******************************
     // *** Define sinusoidal motion ***
     // *******************************


     time_ = time_ + ts_;
     switch (mode)
     {
     case SINUS:
         stPos[0] = (128 + ((int) (256*ampl_))*sin(2*3.14*freq_*time_))/256;
         break;
     //case PORT_BASIC, PORT_ADVANCED:
         //TODO add port reading here!
     case OPERATION_DRIVEN:
            // stPos is filled in by calling operations
             break;
     default:
         break;
     }



     // *****************************
     // *** Write data to gripper ***
     // *****************************

     ((out_Robotiqt*) (m_datap->outputs))->ActionRequest = action_bits.to_ulong();
     ((out_Robotiqt*) (m_datap->outputs))->GripperOptions = option_bits.to_ulong();

     for (int i = 0; i<4; i++){
        ((out_Robotiqt*) (m_datap->outputs))->Fingers[i].PositionFinger = ((int) (256*stPos[i]));
        ((out_Robotiqt*) (m_datap->outputs))->Fingers[i].SpeedFinger = ((int) (256*stVel[i]));
        ((out_Robotiqt*) (m_datap->outputs))->Fingers[i].ForceFinger = ((int) (256*stForce[i]));

     }
     */
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

