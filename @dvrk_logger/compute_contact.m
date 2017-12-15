function compute_contact(self,varargin)
%%  Compute contact locations and contact flags
%   Parse optional inputs
force_thresh = 0.1;
r_ball = 6.30/2; % unit in [mm]
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'force thresh')
            force_thresh = propertyValue;
        elseif strcmp(propertyName,'r ball')
            r_ball = propertyValue;
        end
    end
end
fprintf('Computing all of the contact points in the exploration ...');
reverseStr = [];
N_logs = size(self.logs,2);
for i = 1:N_logs
    msg = sprintf('%0.0f log of %0.0f ... ',i,N_logs);
    fprintf([reverseStr, msg]);
    reverseStr = repmat(sprintf('\b'), 1, length(msg));
    %   info from the exploration
    N_samples = length(self.logs(i).time);
    force = self.logs(i).force;
    position = self.logs(i).position;
    %   compute contact information
    force_mag = sqrt(sum(force.^2));
    contact_flags = (force_mag>force_thresh);
    force_dir = zeros(3,N_samples);
    force_dir(:,contact_flags) = normc(force(:,contact_flags));
    %   apply offest in force direction
    contact_pos = position - force_dir*r_ball;
    %   append the results to log
    self.logs(i).contact_flags = contact_flags;
    self.logs(i).contact_pos = contact_pos;
    self.logs(i).surf_normal = force_dir;
end
fprintf('[ok]\n');
end
