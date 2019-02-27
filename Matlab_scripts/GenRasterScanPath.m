function PATH = GenRasterScanPath(varargin)
%%  Generate Raster Scan Path
%   Long Wang, 2016/9/17
%%  Example usage (either method is ok):
% raster1=GenRasterScanPath('border',x);
% or
% raster2=GenRasterScanPath('input mat','TestBorder');
%%  Inputs
%  borderPts -      a matrix defining the boundaries of a polygon
%                   should be N x 3
%   border_mat -    the mat file name of the Map corners and heights
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
trajLen = 500;
scanningDirXMajor=1;
drawPlan=1;
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'spacing dx')
            spacingDx = propertyValue;
        elseif strcmp(propertyName,'border')    
            borderPts =propertyValue;
        elseif strcmp(propertyName,'input mat')
            border_mat = propertyValue;
        elseif strcmp(propertyName,'path name')
            PathName = propertyValue;
        elseif strcmp(propertyName,'path height')
            mapHeight = propertyValue;
        elseif strcmp(propertyName,'traj len')
            trajLen = propertyValue;
         elseif strcmp(propertyName,'xMajor')
            scanningDirXMajor=propertyValue;
        elseif strcmpi(propertyName,'draw')
            drawPlan=propertyValue;
        elseif strcmp(propertyName,'cycloid')
            cycloid = propertyValue;
        end
    end
end

%%  Load the corners and heights of the map
if exist('border_mat','var')
    Map_Data_Load = load(border_mat);
    MapRefCorners = Map_Data_Load.MapRefCorners;
    MapRefHeights = Map_Data_Load.MapRefHeights;
elseif exist('borderPts','var')
    if size(borderPts,1)~=3
        borderPts=borderPts';
    end
    MapRefCorners=borderPts(1:2,:)';
    MapRefHeights=borderPts(3,:)';
else
    error('Missing input to a border or path to a border mat file');
end

if exist('mapHeight','var')
    MapRefHeights=mapHeight;
end

%%  Calculate points
PointsXY = GenPolyGridPoints(MapRefCorners(:,1),MapRefCorners(:,2),spacingDx,'xMajor',scanningDirXMajor);
PointsZ = max(MapRefHeights)*ones(length(PointsXY),1);
ControlPts = [PointsXY';PointsZ'];

%%  Calculate PATH.SegLen
[~,seglen] = arclength(ControlPts(1,:),ControlPts(2,:),ControlPts(3,:));
seglen = [0,seglen'];
scaled_distance = cumsum(seglen,2);

%% Reinterpolate evenly for the desired number of control points
interpolatedDistance=linspace(0,max(scaled_distance),trajLen);
x=interp1(scaled_distance,ControlPts(1,:),interpolatedDistance);
y=interp1(scaled_distance,ControlPts(2,:),interpolatedDistance);
z=interp1(scaled_distance,ControlPts(3,:),interpolatedDistance);

%% Overlay a cyloidal scan pattern if desired
if exist('cycloid','var')
    a=cycloid.D/(2*pi);
    phi=interpolatedDistance'/a;

    b=a*cycloid.shapeParam;
    offsetX=b.*sin(phi);
    offsetY=b.*cos(phi);

    newX=x+offsetX';
    newY=y+offsetY';
    x=newX;
    y=newY;
    [~,seglen] = arclength(x,y,z);
    seglen=[0,seglen'];
    interpolatedDistance = cumsum(seglen,2);
end

%%  Create a PATH struct format
PATH.POINTS=[x;y;z];
PATH.DATA = [interpolatedDistance;PATH.POINTS];
PATH.PtsNumber = trajLen;
PATH.SCALING = ones(1,PATH.PtsNumber);

if drawPlan
    DrawPathPlan(PATH.DATA);
end

%%  Save the path
save(PathName,'PATH');
end

