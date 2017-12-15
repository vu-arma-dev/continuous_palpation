function PATH = GenTrajectoryPath(PointsXY,MapRefHeight,varargin)
%%  Generate Trajectory Path that can be loaded to xPC
%   Long Wang, 2017/9/5
%%  Properties of struct PATH
%   PATH.PtsNumber  ->  Total number of points
%   PATH.POINTS     ->  (3 X N) matrix all point coord
%   PATH.SegLen     ->  (1 X N) line segment length from the previous point to
%                       current one, i.e. the first segment is always 0.
%   PATH.SCALING    ->  (1 X N) matrix the scaling applied on all segment length, this
%                       will actually control the velocity profile, if no
%                       scaling, value is 1.
%   PATH.DATA       ->  (4 X N) This is Path plan data that parkerXYZ can take in.
%                        the first row is scaled distance and the other
%                        three are coordinates
drawTrajectory= 'off';
trajectoryName = 'temp';
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'draw')
            drawTrajectory = propertyValue;
        elseif strcmp(propertyName,'trajName')
            trajectoryName = propertyValue;
        end
    end
end

%%  Create a PATH struct format
PATH.PtsNumber = 1000;
PATH.POINTS = zeros(3,PATH.PtsNumber);
PATH.SegLen = zeros(1,PATH.PtsNumber);
PATH.SCALING = ones(1,PATH.PtsNumber);
PATH.DATA = zeros(4,PATH.PtsNumber);
%%  Calculate PATH.POINTS
PointsZ = MapRefHeight*ones(size(PointsXY,1),1);
ControlPts = [PointsXY';PointsZ'];
NumControlPts = size(ControlPts,2);
PATH.POINTS(:,1:NumControlPts) = ControlPts;
PATH.POINTS(:,NumControlPts+1:end) = ...
    repmat(ControlPts(:,end),1,PATH.PtsNumber - NumControlPts);
%%  Calculate PATH.SegLen
[~,seglen] = arclength(PATH.POINTS(1,:),PATH.POINTS(2,:),PATH.POINTS(3,:));
PATH.SegLen = [0,seglen'];
scaled_distance = cumsum(PATH.SegLen.*PATH.SCALING,2);
scaled_distance(NumControlPts:end) = ...
    linspace(scaled_distance(NumControlPts),...
    scaled_distance(NumControlPts)+1,...
    PATH.PtsNumber-NumControlPts+1);
% this will ensure the distance values are monotomic
%%  Calculate PATH.DATA
PATH.DATA = [scaled_distance;PATH.POINTS];
%%  [Optional] Draw or Save
if strcmp(drawTrajectory,'on')
    DrawPathPlan(PATH.DATA);
end
if ~strcmp(trajectoryName,'temp')
    Config_mat_path = [getenv('PSMCMD'),'/Config_Mat'];
    Traj_mat_path = [Config_mat_path,'/Trajectories'];
    if ~exist(Traj_mat_path,'dir')
        mkdir(Traj_mat_path);
    end
    save([Traj_mat_path,filesep,trajectoryName],'PATH');
end

