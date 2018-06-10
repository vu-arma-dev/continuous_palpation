## Running Automatic Palpation ##
* roscore
* For each of the following script, need to source the workspace

* dvrk-ros utility:
	* rosrun dvrk_robot dvrk_console_json -j /home/arma/catkin_ws_nico/src/cisst-saw/sawIntuitiveResearchKit/share/vu-dVRK/console-PSM1.json
	* rosrun atinetft_ros atinetft_xml -c /home/arma/catkin_ws_nico/src/cisst-saw/sawATIForceSensor/examples/FT15360Net.xml -i 192.168.1.145
	
* Matlab Utility to define raster trajectory
	* update your "dvrk_init_continous_palp.m", folder path
	* define the raster trajectory, using "DefineExplorationMapCorners.m"

* ipython continuous_palpation.py 
* rosbag record -a -O bagname
* run "Main_RAL_Continuous_Palpation.m", replacing the trajectory name with the defined one.

## Parse RosBag into dvrk logger file format ##
Under construction.

## Digitizing points for Fiducial Update ##
 * roscore
 * Source the workspace
 * dvrk-ros utility:
	* rosrun dvrk_robot dvrk_console_json -j /home/arma/catkin_ws_nico/src/cisst-saw/sawIntuitiveResearchKit/share/vu-dVRK/console-PSM1.json
  * open matlab
  * Update the path commands in dvrk_init_continous_palp to match the dvrk-ros installation folder on your machine
  * Run FiducialLocations.m from MATLAB and collect points A, B, C, D in the proper order
