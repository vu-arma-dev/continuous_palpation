%%  Raster Polygon Grid Points Generation
%   By Long Wang, 2016/9/16
%   This func is modified from Copyright (c) 2013, Sulimon
%%  The following comments are from Sulimon's version
%   inPoints = getPolygonGrid(xv,yv,ppa) returns points that are within a
%   concave or convex polygon using the inpolygon function.

%   xv and yv are columns representing the vertices of the polygon, as used in
%   the Matlab function inpolygon

%   ppa refers to the points per unit area you would like inside the polygon.
%   Here unit area refers to a 1.0 X 1.0 square in the axes.
%%  This code change the original squence to be raster.
function [inPoints] = GenPolyGridPoints(xv,yv,dx,varargin)
if length(dx)==1
    inc_x = dx;
    inc_y = dx;
else
    inc_x = dx(1);
    inc_y = dx(2);
end
%%  parse optional inputs
scanMode = 'raster';
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'scan mode')
            scanMode = propertyValue;
        end
    end
end
%%   Find the bounding rectangle
lower_x = min(xv);
higher_x = max(xv);
lower_y = min(yv);
higher_y = max(yv);
%%  Create a grid of points within the bounding rectangle
interval_x = lower_x:inc_x:higher_x;
interval_y = lower_y:inc_y:higher_y;
[bigGridX, bigGridY] = meshgrid(interval_x, interval_y);
%%  generate raster scan in scanning in X direction
bigGridX = bigGridX';
bigGridY = bigGridY';
if strcmp(scanMode,'raster')
    bigGridX(:,1:2:end) = flip(bigGridX(:,1:2:end),1);
end
%   Filter grid to get only points in polygon
in = inpolygon(bigGridX(:), bigGridY(:), xv, yv);
%   Return the co-ordinates of the points that are in the polygon
inPoints = [bigGridX(in), bigGridY(in)];
end

