clc;close all;clear;
try
    rosshutdown();
catch
end

%% dvrk Init
addpath('~/catkin_ws/src/cisst-saw-nri/dvrk-ros/dvrk_matlab/');
%%
try
    rosinit();
catch
    warning('Using existing ros node');
end
dvrk = psm('PSM2');
%%
force_sub = rossubscriber('/atinetft/wrench');
traj_sub = rossubscriber('/trajectory_length');
rate = rosrate(200);

%% Get corners if necessary
% corners = zeros(3,4);
% for i = 1:4
%     input('go to next corner');
%     p = dvrk.get_position_current();
%     corners(:,i) = p(1:3,4);
% end
% 
% save('4corners.mat','corners')

%% Initialization
load 4corners.mat

%% Send trajectory
    traj_XY = xs_traj(1:2,:);%2*N trajectory of 2D positions
    [X,Y]=meshgrid([15:10:55],[15:10:55]);

    xtmp=reshape(X,size(X(:),1),1);
    ytmp=reshape(Y,size(Y(:),1),1);
    traj_XY = [xtmp,ytmp]';
    depth = -10 ;%should be in mm %should be positive

    traj_dvrk = find_traj_dvrk_frame(traj_XY(1:2,:),corners,depth);
    
    send_trajectory(traj_dvrk, dvrk);
    
    reset(rate)
    results = [];
    t = tic;
    while traj_sub.LatestMessage.Data > 0 || toc(t)<1
        display('.......Trajectory Executing .....')
        f = force_sub.LatestMessage.Wrench.Force;
        p = dvrk.position_current_subscriber.LatestMessage.Pose.Position;
        results = [results; f.X, f.Y, f.Z, p.X, p.Y, p.Z];
        waitfor(rate);
    end
%     for i = 1:size(results,1)
%        results(i,1:3) = dot(results(i,1:3),z_normal);
%     end