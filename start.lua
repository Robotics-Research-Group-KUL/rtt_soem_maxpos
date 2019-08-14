
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
ros:import("rtt_rospack")

ros:import("soem_master")
ros:import("rtt_soem_maxpos")



rtt_soem_maxpos_dir=ros:find("rtt_soem_maxpos")


--rtt.setLogLevel("Info")


depl:loadComponent("Master","soem_master::SoemMasterComponent")



Master=depl:getPeer("Master")

Master:getProperty("ifname"):set("enxe4b97ab11172")

Master:setPeriod(0.001)

Master:configure()

s=Master:provides("Slave_1001")
set_mode_of_operation=s:getOperation("set_mode_of_operation")
reset=s:getOperation("reset")
switch_on=s:getOperation("switch_on")
fault_reset=s:getOperation("fault_reset")
enable_operation=s:getOperation("enable_operation")
disable_operation=s:getOperation("disable_operation")



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

target_position = rttlib.port_clone_conn(s:getPort("target_position"))
target_velocity = rttlib.port_clone_conn(s:getPort("target_velocity"))
target_torque = rttlib.port_clone_conn(s:getPort("target_torque"))


s:getProperty("downsample"):set(100) -- write a sample on 100 in ros ports.
--my motor's value
s:getProperty("encoder_thick_per_revolution"):set(2048)
s:getProperty("gear_ratio"):set(75.0/19.0)

os.execute("sleep 0.1")
Master:start()

--[[
to bring to operational state:


    fault_reset()
    reset()
    set_mode_of_operation(9)--velocity
    switch_on()
     enable_operation()--warning!!! motor will move depending of your setpoints. they are initialized to zero

--]]
