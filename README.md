# continuous_palpation

## Download the needed repositories
* git clone https://github.com/jhu-cisst/cisst-saw --recursive
* git submodule foreach git checkout devel
* git submodule foreach git pull
* git submodule foreach git submodule init
* git submodule foreach git submodule update

* git clone https://github.com/jhu-dvrk/dvrk-ros.git


## To set up the environment
* Make a workspace WORKSPACE_NAME
* make a src directory
* catkin init
* catkin config --profile release -x _release
* catkin profile set release
* catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
* catkin build
* cd ~/WORKSPACE_NAME

## To run the code
* roscore
--	keep running in the background	
* source devel_release/setup.bash
* qlacloserelays
* start the robot console application: rosrun dvrk_robot dvrk_console_json -j /home/arma/catkin_ws_nico/src/cisst-saw/sawIntuitiveResearchKit/share/vu-dVRK/console-PSM1.json
* rosrun atinetft_ros atinetft_xml -c /home/arma/catkin_ws_nico/src/cisst-saw/sawATIForceSensor/examples/FT15360Net.xml -i 192.168.1.145
## From your catkin workspace:
* cd src/continuous_palpation/
* ipython continuous_palpation.py 
* 