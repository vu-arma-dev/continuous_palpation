function PATH = GenRasterScanPath(MapRefName,varargin)
%%  Generate Raster Scan Path
%   Long Wang, 2016/9/17
%%  Example usage:
%   GenRasterScanPath('ExplrMapRef','path name','ExplrMapRefRaster');
%%  Input
%   MapRefName -    the mat file name of the Map corners and heights
%                   inside the mat file, you should see two variables:
%                   MapRefCorners (N by 3), MapRefHeights(N by 1, or 1 by 1)
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
%%  Parse Optional inputs
spacingDx = 1/1000; % [meters]
PathName = 'RasterPath';
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'spacing dx')
            spacingDx = propertyValue;
        elseif strcmp(propertyName,'path name')
            PathName = propertyValue;
        elseif strcmp(propertyName,'xMajor')
            scanningDirXMajor=propertyValue;
        end
    end
end
%%  Load the corners of the map
Config_mat_path = ...
    [getenv('CONT_PALP_DIR'),filesep,...
    'GenPath',filesep,'Config_Mat'];
Map_Data_Load = load([Config_mat_path,filesep,MapRefName]);
MapRefCorners = Map_Data_Load.MapRefCorners;
MapRefHeights = Map_Data_Load.MapRefHeights;
%%  Create a PATH struct format
PATH.PtsNumber = 500;
PATH.POINTS = zeros(3,PATH.PtsNumber);
PATH.SegLen = zeros(1,PATH.PtsNumber);
PATH.SCALING = ones(1,PATH.PtsNumber);
PATH.DATA = zeros(4,PATH.PtsNumber);
%%  Calculate points
NumControlPts = PATH.PtsNumber+1;
while NumControlPts>PATH.PtsNumber
    PointsXY = GenPolyGridPoints(MapRefCorners(:,1),MapRefCorners(:,2),spacingDx,'xMajor',scanningDirXMajor);
    PointsZ = mean(MapRefHeights)*ones(length(PointsXY),1);
    ControlPts = [PointsXY';PointsZ'];
    NumControlPts = size(ControlPts,2);
    spacingDx = sqrt(1.05)*spacingDx;
end
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
DrawPathPlan(PATH.DATA);
%%  Save the path
save([Config_mat_path,'/',PathName],'PATH');
end

