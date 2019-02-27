%%  Trajectory Sending to Continuous Palpation or Path Following
clc;close all;clear;
dvrk_init_continous_palp;
dvrk = nripsm('PSM2');

%%  Ros topics
traj_sub = rossubscriber('/trajectory_length');

%%  Load a raster trajectory
% RasterTrajName='Map2018-08-20';
method='cont';
if strcmp(method,'raster')
    [path]=dvrk_accuracy_test_traj_gen();
elseif strcmp(method,'helix')
    [~,path]=dvrk_accuracy_test_traj_gen();
    path=path';
else
    RasterTrajName = 'kidneySept17Raster';
%     RasterTrajName = 'Sep3test';
    Config_mat_path = ...
    [getenv('CONT_PALP_DIR'),filesep,...
    'Matlab_scripts',filesep,'Config_Mat'];
    LoadedData = load([Config_mat_path,filesep,RasterTrajName]);
    path = LoadedData.PATH.POINTS;
%     path(3,:)=-0.1081; %set map height
end

TotalLengthTraj = size(path,2);
currentTrajSentIdx = 0;
trajSize=5;
nextTrajSentIdx = currentTrajSentIdx + trajSize;

%%  logging while robot in control
fprintf('\nTrajectory being sent ...\n');
trajPub=init_trajectory();
while nextTrajSentIdx<=TotalLengthTraj
    send_trajectory(path(:,currentTrajSentIdx+1:nextTrajSentIdx),dvrk,trajPub);

    while isempty([traj_sub.LatestMessage.Data])% Wait for trajectory to be updated on slave
    end
    currentTrajSentIdx = nextTrajSentIdx;
    nextTrajSentIdx = currentTrajSentIdx + trajSize;
end
fprintf(' [ok].')


