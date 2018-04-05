function DefinePlatformFiducials(FiducialName)
%%  Define the surface exploration map boundary
%   Long Wang, 2017/12/15
%   By selecting the points on the boundary of a polygon,
%   this func
%       1) let the user select the boundary points and save them using
%       MapName
%       2) call "GenRasterScanPath" to generate raster scan trajectory
%   Note that before running this function, need to init dvrk by:
%       1) add the matlab interface folder path
%       2) run the dvrk console program
%%  Input
%   MapName  -    The name to give for this map
if nargin<1
    FiducialName = input('Give fiducial name name:(enter to be default "FiducialRobot")','s');
    if isempty(FiducialName)
        FiducialName = 'FiducialRobot';
    end
end
dvrk_init_continous_palp;
dvrk = psm('PSM1');
DefineNextPoint = 1;
SaveResult = 0;
clc;
fprintf('Manually move the PSM to a point and \n');
fprintf('[empty] - save this point \n');
fprintf('[s] - save all the points and quit  \n');
fprintf('[q] - quit without saving previous points \n');
N = 30;
FiducialPositions = zeros(N,3);
idx = 1;
while DefineNextPoint
    KeyInput = input('Select:','s');
    switch KeyInput
        case ''
            [p,~,~] = getRobotData(dvrk);
            FiducialPositions(idx,:) = p;
            fprintf('Fiducial point %0.0f added.\n',idx);
            idx = idx + 1;
        case 's'
            SaveResult = 1;
            DefineNextPoint = 0;
        case 'q'
            DefineNextPoint = 0;
    end
end
if SaveResult==1
    FiducialPositions(idx:end,:) =[];
    save_mat_path = ...
        [getenv('CONT_PALP_DIR'),filesep,...
        'Data',filesep,'Fiducials'];
    if ~exist(save_mat_path,'dir')
        mkdir(save_mat_path);
    end
    save([save_mat_path,'/',FiducialName],'FiducialPositions');
end
end

