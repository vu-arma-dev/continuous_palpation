function dvrk_init_continous_palp()
%%  checking ros 
try
    rosshutdown();
catch
end

%% dvrk Init
addpath('/home/arma/catkin_ws/src/cisst-saw-nri/nri-ros/dvrk_nri_matlab');
try
    rosinit();
catch
    warning('Using existing ros node');
end
%%  set directory for palpation folder
setenv('CONT_PALP_DIR','/home/arma/catkin_ws/src/continuous_palpation');
end