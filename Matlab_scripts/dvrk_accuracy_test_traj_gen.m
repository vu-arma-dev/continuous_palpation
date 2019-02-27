% Create raster scan for PSM 'calibration' and accuracy test

function [finalPath, otherPath]=dvrk_accuracy_test_traj_gen(filename)
if nargin<1
    filename='Map2018-08-20';
end

Config_mat_path = ...
    [getenv('CONT_PALP_DIR'),filesep,...
    'Matlab_scripts',filesep,'Config_Mat'];
load([Config_mat_path,filesep, filename])

N=4;
baseHeight=-0.1081;
finalHeight=-0.12;
heights=linspace(baseHeight,finalHeight,N);
finalPath=[];
for i=1:N
    raster=GenRasterScanPath('border',[MapRefCorners,repmat(MapRefHeights,size(MapRefCorners,1),1)],...
        'spacing dx',10/1000,'traj len',25,'path height',heights(i),'draw',0); 
    finalPath=[finalPath,raster.POINTS];    
end
N=100;
centerPt=mean(MapRefCorners);
radius=mean(sqrt(sum((MapRefCorners-centerPt).^2,2)))*0.5;
theta=linspace(0,8*pi,N);
z=linspace(baseHeight,finalHeight,N);
otherPath= [cos(theta')*radius+centerPt(1), sin(theta')*radius+centerPt(2),z'];


end