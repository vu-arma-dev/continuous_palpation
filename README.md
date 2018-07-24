# continuous_palpation

## Download the needed repositories
### Full download
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
	* add an origin-arma to have arma force sensor specifics, "git remote add origin-arma https://github.com/wanglong06/sawATIForceSensor.git" 
	* do "git branch VU", then "git pull origin-arma VU"
	* **If you are not at VU, you still need to edit atinetft_xml.cpp to allow for roslaunch capability** see https://github.com/wanglong06/sawATIForceSensor/blob/VU/ros/src/atinetft_xml.cpp copy lines 49-53
* build it using "Compiling" instructions below

* cd WORKSPACE_NAME/src
* git clone https://github.com/jhu-dvrk/dvrk-ros.git
* cd dvrk-ros, 
* git checkout devel
* git pull
* catkin build
* git clone https://github.com/wanglong06/continuous_palpation.git

### Partial download (already have nri)
* cd WORKSPACE_NAME/src
* git clone https://github.com/wanglong06/continuous_palpation.git

## Compiling (first time setup)
* cd to base dir "catkin_ws_xxxx"
* catkin init
* catkin config --profile release -x _release
* catkin profile set release
* catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
* catkin build

## To run the code ##
### First time setup
* Open MATLAB to the continuous_palpation folder. Open Matlab_scripts/dvrk_init_continuous_palp.m
* Edit the line defining the dvrk_nri_matlab path if necessary (line 9)

### Define palpation boundary
* Turn on the robot (presumably launching teleop would work too, but I used this) eg:
```
roscore
rosrun dvrk_robot dvrk_console_json -j /home/arma/catkin_ws/src/cisst-saw-nri/sawIntuitiveResearchKit/share/vu-dVRK/console-PSM2_VU.json
```
* Home the robot
* In MATLAB, run 'GenPath/DefineExplorationMapBoundary.m' and follow directions
* If you want to use the map you just saved, make sure to edit 'Matlab_scripts/Main_RAL_Continuous_Palpation.m' to set the 'RasterTrajName' variable

### Running continuous palpation
* Turn on force sensor if needed (VU needs to run xPC force sensor)
* source your workspace
```
roslaunch continuous_palpation continuous_palpation.launch
```
* In MATLAB, run Main_RAL_Continuous_Palpation
	* Make sure to set RasterTrajName to whatever mat file you collected above for the trajectory boundary
