function [pos,quat,force]=getRobotData(dvrk,force_sub)
%%  get robot data
%   position's unit is in [Meter]
%   orientation is using quaternion to represent
%   force's unit is [Newton]
if nargin<2
    force = nan(3,1);
else
    force = [force_sub.LatestMessage.Wrench.Force.X;
        force_sub.LatestMessage.Wrench.Force.Y;
        force_sub.LatestMessage.Wrench.Force.Z];
end
pos = [dvrk.position_current_subscriber.LatestMessage.Pose.Position.X;
    dvrk.position_current_subscriber.LatestMessage.Pose.Position.Y;
    dvrk.position_current_subscriber.LatestMessage.Pose.Position.Z];
quat = [dvrk.position_current_subscriber.LatestMessage.Pose.Orientation.W;
    dvrk.position_current_subscriber.LatestMessage.Pose.Orientation.X;
    dvrk.position_current_subscriber.LatestMessage.Pose.Orientation.Y;
    dvrk.position_current_subscriber.LatestMessage.Pose.Orientation.Z];
end