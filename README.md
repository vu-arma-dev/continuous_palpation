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





