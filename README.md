# AIRo Control Interface
This project provides a quadrotor UAV control interfaces based on PX4 that uses customized outter-loop controllers. Currently supported controllers include Model Predictive Control (MPC), Backstepping Control (BS), and Sliding-Mode Control(SMC). By using this package you can have functions such as auto takeoff/land, RC transmitter control, and follow external trajectory commands with customized outter-loop controllers.

## Installation

It is recommanded to run the package in our docker following instructions ([here](https://github.com/HKPolyU-UAV/docker_practice)). By doing so, you can skip the installation section.

** Prerequisites **
* ROS ([ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) recommended)
* [QGroundControl](http://qgroundcontrol.com/)
* [MAVROS](http://wiki.ros.org/mavros)
* [Acados](https://docs.acados.org/installation/index.html)

First, install Acados at your home directory. If you want to install Acados at other directory, change the acados_include and acados_lib directory written in CMakeLists.txt of airo_control package, and also change ```~/acados``` in the following codes to your customized directory in the following codes.
```
cd ~
git clone https://github.com/acados/acados.git
cd acados
git checkout 568e46c
git submodule update --recursive --init
mkdir build
cd build
cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=~/acados ..
sudo make install -j4
```

Create a catkin workspace and clone this repository to src folder (ex. ~/airo_control_interface_ws/src)
```
mkdir -p ~/airo_control_interface_ws/src
cd ~/airo_control_interface_ws
catkin_make
cd src
git clone https://github.com/HKPolyU-UAV/airo_control_interface.git
```

Build the package.
```
cd ~/airo_control_interface_ws
catkin_make
```

Download and install the PX4 (1.11.0)
```
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot/
git checkout 71db090
git submodule sync --recursive
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
sudo apt upgrade libignition-math4 #(libignition-math2 for melodic)
```

Add acados library path to ```.bashrc```, change ```<path_to_acados>``` to your path, e.g. ```/home/your_username```. Note that do not use ```~``` in ```<path_to_acados>```.
```
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"<path_to_acados>/acados/lib"' >> ~/.bashrc
echo 'export ACADOS_SOURCE_DIR="<path_to_acados>/acados"' >> ~/.bashrc
```

Add source ```setup.bash``` file to ```.bashrc```.
echo 'source ~/airo_control_interface_ws/devel/setup.bash' >> ~/.bashrc
```

## Running Simulation

Start PX4 SITL
```
cd ~/PX4-Autopilot/
make px4_sitl_default gazebo
```

Run MAVROS
```
roslaunch airo_control mavros_gazebo.launch
```


Start control interface in new terminal
```
roslaunch airo_control fsm_gazebo.launch
```

Run an example mission
```
rosrun airo_control example_mission_node
```

Or, you can simply run start.sh in the startup folder.

## Usage with RC Transmitter

You can use this control interface with an RC transmitter for both gazebo simulation and real-world deployment.

** Hardware Setup **
First, launch mavros and QGC and make sure the quadrotor is connected. To use the interface in gazebo simulation, set PX4 parameter COM_RCIN_MODE to "RC and Joystick with fallback", connect RC transmitter via usb serial, calibrate the joysticks in "Joysticks" tab and you should be able to read channel inputs in QGC "Radio" tab. In QGC setup page, you only need to set emergency kill switch channel and flight mode switch channel. Three more RC channel will be used by the control interface, which are referred to as FSM channel (channel 5 by default), command channel (channel 6 by default), and reboot channel (channel 8 by default). The FSM channel controls if the interface is activated, the command channel controls if the interface follows external trajecdtory commands, and the reboot channel will reboot FCU so that it is recommended to be set to the stick that can automatically flip back. If the pwm output of these three switch channesl is greater than the threshold (1750 by default), the channel is enabled.

** Preparation **
Before using the interface, make sure the vehicle can be used with position flight mode. Then, run the system identification program to determine the hover_thrust, tau_phi, and tau_theta. After this, the FSM should be ready to rock. When vehicle is landed, you can use reboot channel to reboot the FCU. 
In general, it is recommended to choose to enable or disable command channel before running the FSM based on the application scenarios.  Although switching command channel during mission is supported, it is not required during common applications and could cause confusions. Therefore, we recommend the following two pipelines to work with this FSM. 

** Non-command Mode **

To use in non-command mode, first disable the command channel and then switch the FSM channel. The FSM will ask the user to center all joysticks to avoid sudden position change when jump to AUTO_HOVER state. Then, the vehicle should arm and accelerate motors for a few seconds and automatically takeoff to desired height. Once takeoff height is reached, the vehicle will follow the joystick commands. If vehicle has been landed, the FSM will disarm it and reinitiate for the next mission. 

** Command Mode **

To use in command mode, first enable the command channel and then enable the FSM channel. Note that the command mode is capable to be used without RC transmitter by setting parameter ```without_rc``` to true. Then the user can send takeoff trigger ```takeoff_land_trigger = true``` to topic ```/airo_px4/takeoff_land_trigger```. After auto takeoff, the FSM will publish indicator ```is_waiting_for_command = true``` to topic “/airo_px4/fsm_info". By receiving the indicator, the quadrotor will follow commands published to ```/airo_control/setpoint``` (or ```/airo_control/setpoint_preview``` if MPC is used). If you stop sending commands, the fsm will go back to AUTO_HOVER mode. To land and disarm the vehicle, send ```takeoff_land_trigger = false``` to the same topic.

## FSM Introduction

The control interface uses a finite state machine to control the UAV with the detials introduced below.

<img src="media/AIRo_PX4_FSM.png">

1. RC_MANUAL 

In this state, the FSM is disabled and the quadrotor operates at manual modes (i.e. position,altitude, and stabilize) using the embedded PID controllers in PX4 firmware. The FSM is initialized with this state and will go back to it every time the vehicle is disarmed. This is the only state that PX4 offboard is disabled and the user have total control over RC transmitter using the embedded controller in FCU. 

2. AUTO_TAKEOFF 

In this state, the vehicle will perform auto takeoff operation. The vehicle will slowly accelerate motors for several seconds and then takeoff to takeoff_height at takeoff_land_speed. This state can be triggered from RC_MANUAL state in two conditions. First condition is if command channel is disabled, the vehicle is landed, and the FSM channel is switched. Second is if command channel is enabled, FSM is enabled, and takeoff trigger is received. Once the target height is reached, the FSM will jump to AUTO_HOVER state. 

3. AUTO_HOVER 

In this state, the vehicle will follow the commend of RC transmitter joysticks, which is similar to the position flight mode. This state can be triggered after AUTO_TAKEOFF or if the FSM channel switched during manual flight using RC_MANUAL. User can set a safety volume to confine the vehicle position. If the vehicle is landed with joystick commands, the FSM will disarm the vehicle and jump back to RC_MANUAL state. 
If command channel is enabled, the FSM will send "is_waiting_for_command = true" to topic "/airo_px4/fsm_info" to indicate that the vehicle is waiting for external commands. 

4. AUTO_LAND 

In this state, the vehicle will automatically land and disarm at current x&y position. This state can only be triggered when command channel enabled and land command is received. 

5. POS_COMMAND 

In this state, the vehicle will follow external position command subscribed from “/airo_px4/setpoint“.

Note that in all states, the position reference given to the controler is confined by the safety constraints set in ```.yaml``` file

## Generate MPC Solver

The python scripts used to generate MPC solver is included in ```~/catkin_ws/src/airo_control_interface/airo_control/acados_scripts/quadrotor_model.py```. If you want to make modifications and generate MPC solver by your own, follow these instructions.

Install python 3.7
```
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.7
```

Install python dependencies
```
python3 -m pip install pip
sudo pip3 install numpy matplotlib scipy future-fstrings casadi>=3.5.1 setuptools
sudo apt-get install python3.7-tk
pip install -e ~/acados/interfaces/acados_template
```

Generate solver, note that for the first run, you should setup Tera renderer automatically by following the instructions.
```
cd ~/catkin_ws/src/airo_control_interface/airo_control/acados_scripts
python3 generate_c_code.py
```

## MPC System Identification

To use the control framework with MPC, the quadrotor model parameters should first be identified, which includes hover thrust and inner control loop dynamics for pitch, roll, and yaw movements. In order to do this, first make sure that the quadrotor can be used in position flight mode, then launch ```system_id.launch``` in ```airo_control``` package. The quadrotor will automatically takeoff, perform maneuvers in all axes, and land. Once it's landed, the identified parameters will be displayed in terminal and you can save it in ```.yaml``` file by input ```y```.
