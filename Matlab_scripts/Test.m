%%  Automatic Continuous palpation
%   to collecting data for Preetham's RAL 2018
clc;close all;clear;
dvrk_init_continous_palp;
dvrk = nripsm('PSM2');
%%  Ros topics
% force_sub = rossubscriber('/atinetft/wrench');
rate = rosrate(200);
%%  Load a raster trajectory
RasterTrajName='June10RasterOutputTemp';
Config_mat_path = ...
    [getenv('CONT_PALP_DIR'),filesep,...
    'Matlab_scripts',filesep,'Config_Mat'];
LoadedData = load([Config_mat_path,filesep,RasterTrajName]);
PATH = LoadedData.PATH;
TotalLengthTraj = size(PATH.DATA,2);
currentTrajSentIdx = 0;
trajSize=15;
nextTrajSentIdx = currentTrajSentIdx + trajSize;

%%  logging while robot in control

fprintf('\nTrajectory being sent ...\n');
reverseStr = [];
trajPub=init_trajectory();
while nextTrajSentIdx<TotalLengthTraj
    index = 0;
    send_trajectory(PATH.DATA(2:4,currentTrajSentIdx+1:nextTrajSentIdx),dvrk,trajPub);
    currentTrajSentIdx = nextTrajSentIdx;
    nextTrajSentIdx = currentTrajSentIdx + trajSize;
    pause(0.1)
    if nextTrajSentIdx>=TotalLengthTraj
        currentTrajSentIdx = 0;
        nextTrajSentIdx = currentTrajSentIdx + trajSize;
    end
end
fprintf(' [ok].')

%% Finish experiment and save data
