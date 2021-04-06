
require "rttlib"
require "rttros"

----------------------
-- get the deployer --
tc=rtt.getTC()
if tc:getName() == "lua" then
    depl=tc:getPeer("Deployer")
elseif tc:getName() == "Deployer" then
    depl=tc
end


rtt.setLogLevel("Warning")


gs = gs or rtt.provides()

depl:import("rtt_ros")
depl:import("rtt_rosnode")
ros = gs:provides("ros")
ros:import("rtt_roscomm")
ros:import("rtt_std_msgs")
ros:import("rtt_std_srvs")
ros:import("rtt_rospack")


ros:import("soem_master")
ros:import("rtt_soem_maxpos")
ros:import("rtt_maxpos_msgs")



rtt_soem_maxpos_dir=ros:find("rtt_soem_maxpos")


--rtt.setLogLevel("Info")


depl:loadComponent("Master","soem_master::SoemMasterComponent")



Master=depl:getPeer("Master")

Master:getProperty("ifname"):set("enxe4b97ab11172")

Master:setPeriod(0.01)

Master:configure()

s=Master:provides("Slave_1001")
set_mode_of_operation=s:getOperation("set_mode_of_operation")
reset=s:getOperation("reset")
switch_on=s:getOperation("switch_on")
fault_reset=s:getOperation("fault_reset")
enable_operation=s:getOperation("enable_operation")
disable_operation=s:getOperation("disable_operation")
velocity_ramp=s:getOperation("velocity_ramp")


depl:stream("Master.Slave_1001.status",
        rtt.provides("ros"):topic("/status"))
depl:stream("Master.Slave_1001.position_ros",
        rtt.provides("ros"):topic("/position"))
depl:stream("Master.Slave_1001.velocity_ros",
       rtt.provides("ros"):topic("/velocity"))
depl:stream("Master.Slave_1001.torque_ros",
        rtt.provides("ros"):topic("/torque"))
depl:stream("Master.Slave_1001.digital_inputs",
       rtt.provides("ros"):topic("/digital_inputs"))
depl:stream("Master.Slave_1001.touch_probe",
        rtt.provides("ros"):topic("/touch_probe"))
depl:stream("Master.Slave_1001.status_8MSB",
        rtt.provides("ros"):topic("/status_8MSB"))
depl:stream("Master.Slave_1001.modes_of_operation_display",
        rtt.provides("ros"):topic("/modes_of_operation_display"))

depl:loadService("Master","rosservice") 
rs=Master:provides("rosservice")

connect=rs:getOperation("connect")
       
connect("Slave_1001.velocity_ramp_service", "MaxPos/VelocityRamp", 
  "maxpos_msgs/VelocityProfile")
        
connect("Slave_1001.bring_operational_service", "MaxPos/BringOperational", 
  "std_srvs/Empty")

connect("Slave_1001.disable_operation_service", "MaxPos/Disable", 
  "std_srvs/Empty")


target_position = rttlib.port_clone_conn(s:getPort("target_position"))
target_velocity = rttlib.port_clone_conn(s:getPort("target_velocity"))
target_torque = rttlib.port_clone_conn(s:getPort("target_torque"))


s:getProperty("downsample"):set(10) -- write a sample on 100 in ros ports.
--my motor's value
s:getProperty("encoder_thick_per_revolution"):set(1024)
s:getProperty("gear_ratio"):set(1)

os.execute("sleep 0.1")
Master:start()

--[[
to bring to operational state:


    fault_reset()
    reset()
    set_mode_of_operation(9)--velocity
    switch_on()
    enable_operation()
velocity_ramp(60,10)

--]]
