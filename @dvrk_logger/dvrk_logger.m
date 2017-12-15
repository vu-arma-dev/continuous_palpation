classdef dvrk_logger < handle
    %%  dvrk_logger
    %   Created by Nico, 2016/7
    %   Last modifed by Long, 2016/9/20
    %   This class is used to log the data from the PSM robot.
    properties (SetAccess = public)
        logName
        logs
        current_log
        start_time
        plotData  % this is used to store data for visual use
    end
    
    properties (SetAccess = protected)
        current_log_idx
        max_log_length
        datafolder
    end
    
    methods
        function self = dvrk_logger(logName,datafolderin)
            self.logName = logName;
            self.datafolder=datafolderin;
            self.max_log_length = 10000;
            self.current_log = struct('time',zeros(1,self.max_log_length), ...
                'position', zeros(3,self.max_log_length), ...
                'force', zeros(3,self.max_log_length), ...
                'quat', zeros(4,self.max_log_length));
            self.current_log_idx = 1;
            self.start_time = tic;
            self.logs = [];
        end
        function log_position_force_quat(self, position, force, quat)
            if self.current_log_idx > self.max_log_length
                self.continue_log();
            end
            idx = self.current_log_idx;
            timestamp = toc(self.start_time);
            self.current_log.time(idx) =  timestamp;
            self.current_log.position(:,idx) = position(:);
            self.current_log.force(:,idx) = force(:);
            self.current_log.quat(:,idx) = quat(:);
            self.current_log_idx = self.current_log_idx+1;
        end
        function end_log(self)
            idx = self.current_log_idx-1;
            self.logs = [self.logs, struct('time',[],'position',[],'force',[],'quat',[])];
            self.logs(end).time = self.current_log.time(:,1:idx);
            self.logs(end).force = self.current_log.force(:,1:idx);
            self.logs(end).position = self.current_log.position(:,1:idx);
            self.logs(end).quat = self.current_log.quat(:,1:idx);
            self.current_log_idx = 1;
        end
        function continue_log(self)
            if isempty(self.logs)
                self.logs = self.make_new_log();
                self.logs.time = [self.current_log.time];
            else
                self.logs(end+1).time = [self.current_log.time];
            end
            self.logs(end).force = [self.current_log.force];
            self.logs(end).position = [self.current_log.position];
            self.logs(end).quat = [self.current_log.quat];
            self.current_log_idx = 1;
            self.current_log=self.make_new_log;
        end
        %%  Following functions are added by Long, used only for JMR experiments
        compute_contact(self,varargin);
        plot_explr_map(self,varargin);
        gen_explr_video(self,varargin);
        save(self,data_sub_folder);
    end
    methods(Static)
        function result = make_new_log()
            result = struct('time',[],'position',[],'force',[],'quat',[]);
        end
    end
end