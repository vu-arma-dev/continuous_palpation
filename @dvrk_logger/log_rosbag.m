function log_rosbag(self,rosbag_file_path)
%%  parse rosbag and log into logger
%   Long Wang, 2018/2/20
fprintf('Reading rosbag file \n  [%s] \n      ... ',...
    rosbag_file_path);
BAG_SELECT = rosbag(rosbag_file_path);
fprintf('[ok]\n');
[st_Time,end_Time] = getTrajectoryTimeRange();
log_messages(st_Time,end_Time);
%%% END OF THE MAIN FUNCTION %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Nested Functions Below this section
%   Checkout the nested function implementations at:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% getTrajectoryTimeRange()
%   This func extract the [start] and [end] time of the trajectory
    function [startTime, endTime] = getTrajectoryTimeRange()
        fprintf('Segmenting the trajectory ... \n');
        % find startTime using '/set_continuous_palpation_trajectory'
        bagselect_SendTraj = select(BAG_SELECT, ...
            'Topic', '/set_continuous_palpation_trajectory');
        commandedTime = bagselect_SendTraj.timeseries.Time;
        startTime = min(commandedTime);
        lastCommandTime = max(commandedTime);
        % find endTime using '/trajectory_length' and the lastCommandTime
        bagselect_TrajLen = select(BAG_SELECT,...
            'Time',[lastCommandTime,BAG_SELECT.EndTime],...
            'Topic','/trajectory_length');
        TrajLenMsg = bagselect_TrajLen.readMessages;
        TrajLen_cell = cellfun(@(x) x.Data,TrajLenMsg);
        TrajTime = bagselect_TrajLen.timeseries.Time;
        endTime_Index = find(TrajLen_cell>1,1,'last') + 1;
        endTime = TrajTime(endTime_Index);
        fprintf('    Start time: %.2f\n',startTime);
        fprintf('    End time: %.2f\n',endTime);
        fprintf('[ok]\n');
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   log_messages
    function log_messages(startTime,endTime)
        fprintf('Logging samples from rosbag to dvrk_logger ... \n');
        reverseStr = [];
        time_interval = 1; % unit in seconds
        inLoopStart = startTime;
        while (endTime > inLoopStart)
            if (endTime-inLoopStart)>time_interval
                inLoopEnd = inLoopStart + time_interval;
            else
                inLoopEnd = endTime;
            end
            % pose bag
            bag_pose = select(BAG_SELECT,...
                'Time',[inLoopStart,inLoopEnd],...
                'Topic',...
                '/dvrk/PSM1/position_cartesian_current');
            pose_Time = bag_pose.timeseries.Time;
            % wrench bag
            bag_wrench = select(BAG_SELECT,...
                'Time',[inLoopStart,inLoopEnd],...
                'Topic',...
                '/atinetft/wrench');
            wrench_Time = bag_wrench.timeseries.Time;
            % fill in dvrk_logger
            %   we use the pose_Time as the reference
            N_time = length(pose_Time);
            for t_i = 1:N_time
                % get the msg of pose at t_i time
                bag_pose_msg =bag_pose.readMessages(t_i);
                position = [...
                    bag_pose_msg{1}.Pose.Position.X,...
                    bag_pose_msg{1}.Pose.Position.Y,...
                    bag_pose_msg{1}.Pose.Position.Z];
                quat = [...
                    bag_pose_msg{1}.Pose.Orientation.W,...
                    bag_pose_msg{1}.Pose.Orientation.X,...
                    bag_pose_msg{1}.Pose.Orientation.Y,...
                    bag_pose_msg{1}.Pose.Orientation.Z];
                % find the closest time index in wrench
                [~,t_i_wrench] = ...
                    min(abs(pose_Time(t_i) - wrench_Time));
                bag_wrench_msg = bag_wrench.readMessages(t_i_wrench);
                force = [...
                    bag_wrench_msg{1}.Wrench.Force.X,...
                    bag_wrench_msg{1}.Wrench.Force.Y,...
                    bag_wrench_msg{1}.Wrench.Force.Z];
                self.log_position_force_quat(...
                    position,...
                    force,...
                    quat,...
                    pose_Time(t_i)...
                    );
                %   display current progress
                displayMsg = sprintf(...
                    'current processed time %.2f [sec] / %.2f [sec] ... ',...
                    pose_Time(t_i) - startTime,...
                    endTime-startTime);
                fprintf([reverseStr, displayMsg]);
                reverseStr = repmat(sprintf('\b'), 1, length(displayMsg));
            end
            inLoopStart = inLoopEnd;
        end
        fprintf('[ok]\n');
    end
end