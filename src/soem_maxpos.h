

#ifndef SOEM_MAXPOS_H
#define SOEM_MAXPOS_H

#include <soem_master/soem_driver.h>
#include <bitset>





class SoemMaxPos : public soem_master::SoemDriver
{

    typedef struct PACKED
    {
        uint16 status_word;
        int32  position_actual_value;
        int32  velocity_actual_value;
        int16  torque_actual_value;
        uint32  digital_inputs;
        uint16 touch_probe_status;
    } in_1a00;

    typedef struct PACKED
    {
        uint8 ActionRequest;
        uint8 GripperOptions;
        uint8 GripperOptions2;
        out_Robotiqt_Finger Fingers[4];
        uint8 Reserved;
    } out_1a00;
    
public:
    SoemMaxPos(ec_slavet* mem_loc);
    ~SoemMaxPos(){}

    bool configure();
    void update();



private:

};

#endif
