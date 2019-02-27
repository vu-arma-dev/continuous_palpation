function send_trajectory( positions, robot,publisher)
%SEND_TRAJECTORY Sends a trajectory message to a continuous palpation node
%   Positions is a 3xN matrix of positions. The current orientation is kept
%   constant.
    msg = rosmessage(publisher);
    msg.Header.Stamp = rostime('now');
    curr = robot.position_current_subscriber.LatestMessage.Pose;
    for idx = 1:size(positions,2)
        pose = curr.copy();
        pose.Position.X = positions(1,idx);
        pose.Position.Y = positions(2,idx);
        pose.Position.Z = positions(3,idx);
        msg.Poses = [msg.Poses; pose];
    end
    send(publisher, msg);
end

    