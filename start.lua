
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

Master:setPeriod(0.01)

Master:configure()
Master:start()

depl:stream("Master.Slave_1001.Status",
        rtt.provides("ros"):topic("/status"))

