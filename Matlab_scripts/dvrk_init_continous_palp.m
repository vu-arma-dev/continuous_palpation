function dvrk_init_continous_palp()
%%  checking ros 
try
    rosshutdown();
catch
    warning('shutdown failure')
end

%% dvrk Init
addpath('/home/arma/catkin_ws/src/cisst-saw-nri/nri-ros/dvrk_nri_matlab');
try
    rosinit();
catch
    warning('Using existing ros node');
end
%%  set directory for palpation folder
setenv('CONT_PALP_DIR',fileparts(fileparts(mfilename('fullpath'))));

addpath(genpath(getenv('CONT_PALP_DIR')));
addpath(genpath(getenv('ARMA_CL')));
end