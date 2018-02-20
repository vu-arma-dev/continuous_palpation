# continuous_palpation

## Download the needed repositories
* Make a workspace WORKSPACE_NAME and navigate to it
* mkdir src and navigate to it
* git clone https://github.com/jhu-cisst/cisst-saw --recursive
* cd cisst-saw
* git submodule foreach git checkout devel
* git submodule foreach git pull
* git submodule foreach git submodule init
* git submodule foreach git submodule update
* under "cisst-saw/sawRobotIO1394/components/code/Amp1394", do "git checkout devel", "git pull"
* under "cisst-saw/sawATIForceSensor/", 
	* add an origin-arma to have arma force sensor specifics, "git remote add origin-arma https://github.com/wanglong06/sawIntuitiveResearchKit.git" 
	* do "git branch VU", then "git pull origin-arma VU"
* build it using "Compiling" instructions below

* cd WORKSPACE_NAME/src
* git clone https://github.com/jhu-dvrk/dvrk-ros.git
* cd dvrk-ros, 
* git checkout devel
* git pull
* catkin build
* git clone https://github.com/wanglong06/continuous_palpation.git

## Compiling
* cd to base dir "catkin_ws_xxxx"
* catkin init
* catkin config --profile release -x _release
* catkin profile set release
* catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
* catkin build

## To run the code ##

* roscore
--	keep running in the background	
* cd catkin_ws_nico
* source devel_release/setup.bash
* qlacloserelays
* start the robot console application: rosrun dvrk_robot dvrk_console_json -j /home/arma/catkin_ws_nico/src/cisst-saw/sawIntuitiveResearchKit/share/vu-dVRK/console-PSM1.json
* rosrun atinetft_ros atinetft_xml -c /home/arma/catkin_ws_nico/src/cisst-saw/sawATIForceSensor/examples/FT15360Net.xml -i 192.168.1.145

## From your catkin workspace:
* cd src/continuous_palpation/
* ipython continuous_palpation.py