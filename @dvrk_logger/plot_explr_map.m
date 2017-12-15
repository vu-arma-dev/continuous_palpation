function plot_explr_map(self,varargin)
MakeNewFigure= 'on';
MarkerSize = 5;
MarkerColor = 'b';
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'new figure')
            MakeNewFigure = propertyValue;
        elseif strcmp(propertyName,'MarkerSize')
            MarkerSize = propertyValue;
        elseif strcmp(propertyName,'MarkerColor')
            MarkerColor = propertyValue;
        end
    end
end
%%  Plot the exploration result map and store data to plotData
%   Only the collected points that are detected as contacts are
%   plotted.
%%  Note that usually one logger only has two logs.
%   The first log contains huge data
%   The second log is the most recent collection.
%%  Format all seperated N logs to giant 3D matrix
max_log_length = length(self.logs(1).time);
N_logs = size(self.logs,2);
contact_pos_merged_3D = nan(3,max_log_length,N_logs);
surf_normal_merged_3D = nan(3,max_log_length,N_logs);
wrist_quat_merged_3D = nan(4,max_log_length,N_logs);
contact_flags_merged_2D = nan(max_log_length,N_logs);
for i = 1:N_logs
    N_samples = length(self.logs(i).time);
    contact_pos_merged_3D(:,1:N_samples,i) = ...
        self.logs(i).contact_pos;
    surf_normal_merged_3D(:,1:N_samples,i) = ...
        self.logs(i).surf_normal;
    wrist_quat_merged_3D(:,1:N_samples,i) = ...
        self.logs(i).quat;
    contact_flags_merged_2D(1:N_samples,i) = ...
        self.logs(i).contact_flags;
end
%%  Partition 3D matrix to 2D
%   Note that this step needs attention
%   The current conversion is ONLY working for (3 by N) for
%   each log. Because reshape is following columns
contact_pos_merged = reshape(contact_pos_merged_3D,...
    [3,N_logs*max_log_length]);
surf_normal_merged = reshape(surf_normal_merged_3D,...
    [3,N_logs*max_log_length]);
wrist_quat_merged = reshape(wrist_quat_merged_3D,...
    [4,N_logs*max_log_length]);
contact_flags_merged = reshape(contact_flags_merged_2D,...
    [N_logs*max_log_length,1]);
%%  Delete all nan elements in the last log
%   all nan elements are just holders for format purpose
nan_idx = isnan(contact_flags_merged);
contact_flags_merged(nan_idx) = [];
contact_pos_merged(:,nan_idx) = [];
surf_normal_merged(:,nan_idx) = [];
wrist_quat_merged(:,nan_idx) = [];
self.plotData = struct('contact_flags',[],...
    'contact_pos',[],'surf_normal',[],'wrist_quat',[]);
self.plotData.contact_flags = contact_flags_merged;
self.plotData.contact_pos = contact_pos_merged;
self.plotData.surf_normal = surf_normal_merged;
self.plotData.wrist_quat = wrist_quat_merged;
%%  plot
if strcmp(MakeNewFigure,'on')
    figure;
    hold on;
    axis equal;
    view(126,48);
end
scatter3(contact_pos_merged(1,contact_flags_merged==1),...
    contact_pos_merged(2,contact_flags_merged==1),...
    contact_pos_merged(3,contact_flags_merged==1),MarkerSize,MarkerColor);
end
