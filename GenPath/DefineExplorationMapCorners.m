function DefineExplorationMapCorners(MapName)
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
    MapName = input('Give new map name:(enter to be default "ExplrMap")','s');
    if isempty(MapName)
        MapName = 'ExplrMap';
    end
end
dvrk_init_contiuous_palp;
dvrk = psm('PSM1');
DefineNextPoint = 1;
SaveResult = 0;
clc;
fprintf('Manually move the PSM to a point and \n');
fprintf('[empty] - save this point \n');
fprintf('[h] - save the map height using this point \n');
fprintf('[s] - save all the points and quit  \n');
fprintf('[q] - quit without saving previous points \n');
N = 30;
MapRefCorners = zeros(N,2);
MapRefCornersZ = zeros(N,1);
MapRefHeights = nan;
idx = 1;
while DefineNextPoint
    KeyInput = input('Select:','s');
    switch KeyInput
        case ''
            [p,~,~] = getRobotData(dvrk);
            MapRefCorners(idx,:) = p(1:2);
            MapRefCornersZ(idx) = p(3);
            fprintf('Boudary point %0.0f added.\n',idx);
            idx = idx + 1;
        case 'h'
            [p,~,~] = getRobotData(dvrk);
            MapRefHeights = p(3);
            fprintf('Map height defined.\n');            
        case 's'
            SaveResult = 1;
            DefineNextPoint = 0;
            if isnan(MapRefHeights)
                MapRefHeights = mean(MapRefCornersZ);
            end
        case 'q'
            DefineNextPoint = 0;
    end
end
if SaveResult==1
    MapRefCorners(idx:end,:) =[];
    Config_mat_path = ...
        [getenv('CONT_PALP_DIR'),filesep,...
        'GenPath',filesep,'Config_Mat'];
    save([Config_mat_path,'/',MapName],'MapRefCorners','MapRefHeights');
    GenRasterScanPath(MapName,'path name',[MapName,'Raster']);
end
end

