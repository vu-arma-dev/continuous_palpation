%%  Automatic Continuous palpation
%   to collecting data for Preetham's RAL 2018
clc;close all;clear;
dvrk_init_contiuous_palp;
dvrk = psm('PSM1');
myTime=datetime;
logger=dvrk_logger([datestr(myTime,'mmm-dd_HH-MM-SS') '_Continuous'],'/home/arma/catkin_ws_nico/src/continuous_palpation');
%%  Ros topics
force_sub = rossubscriber('/atinetft/wrench');
traj_sub = rossubscriber('/trajectory_length');
rate = rosrate(200);
%%  Load a raster trajectory
%   The raster trajectory is generated using the followign command line
%   DefineExplorationMapCorners
RasterTrajName = 'ExplrMapTestRaster';
Config_mat_path = ...
    [getenv('CONT_PALP_DIR'),filesep,...
    'GenPath',filesep,'Config_Mat'];
LoadedData = load([Config_mat_path,filesep,RasterTrajName]);
PATH = LoadedData.PATH;
TotalLengthTraj = size(PATH.DATA,2);
currentTrajSentIdx = 0;
nextTrajSentIdx = currentTrajSentIdx + 10;

%%  logging while robot in control
i = 0;
fprintf('\nData samples being collected ...\n');
reverseStr = [];
while nextTrajSentIdx<TotalLengthTraj
    send_trajectory(PATH.DATA(2:4,currentTrajSentIdx+1:nextTrajSentIdx),dvrk);
    while traj_sub.LatestMessage.Data>1
        [pos,quat,force]=getRobotData(dvrk,force_sub);
        logger.log_position_force_quat(pos,force,quat);
        msg = sprintf('%.0f samples ... ',i);
        fprintf([reverseStr, msg]);
        reverseStr = repmat(sprintf('\b'), 1, length(msg));
        i = i+1;
    end
    currentTrajSentIdx = nextTrajSentIdx;
    nextTrajSentIdx = currentTrajSentIdx + 10;
end
fprintf(' [ok].')

%% Finish experiment and save data
logger.end_log;
logger.save;