function gen_explr_video(self,varargin)
%%  This func generates video of the exploration
%%  parse optional arguments
videoName = 'Explr_video';
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'video name')
            videoName = propertyValue;
        end
    end
end
%%  Define/Open VideoWriter obj
root_path = fileparts(getenv('PSMCMD'));
if ~exist([root_path,'\Figures_Videos\'],'dir')
    mkdir([root_path,'\Figures_Videos\']);
end
v = VideoWriter([root_path,'\Figures_Videos\',videoName],'MPEG-4');
v.Quality = 100;
%   Calculate fps
time_diff = diff(self.logs(1).time);
fps = 1/mean(time_diff);
v.FrameRate = fps;
open(v);
%
figure;
hold on;
axis equal;
box on;
grid on;
scatter_size = 2.5;
scatter3(self.plotData.contact_pos(1,self.plotData.contact_flags==1),...
    self.plotData.contact_pos(2,self.plotData.contact_flags==1),...
    self.plotData.contact_pos(3,self.plotData.contact_flags==1),scatter_size);
view(115,22);
fprintf('Adjust the view for video generation ... hit any key to continue ... \n');
pause;
[az,el] = view;
view_lim = axis;
N_samples = length(self.plotData.contact_flags);
fprintf('Video being generated ...\n');
reverseStr = [];
for k = 1:N_samples
    if self.plotData.contact_flags(k)==1
        % figure clean up and view set up
        cla;
        view(az,el);
        axis(view_lim);
        %   plot the current exploration map
        points_idx_so_far = 1:k;
        points_idx_so_far_contact = ...
            points_idx_so_far(self.plotData.contact_flags(1:k)==1);
        scatter3(self.plotData.contact_pos(1,points_idx_so_far_contact),...
            self.plotData.contact_pos(2,points_idx_so_far_contact),...
            self.plotData.contact_pos(3,points_idx_so_far_contact),...
            scatter_size);
        %   plot the force direction
        force_arrow_end = 10*self.plotData.surf_normal(:,k)+...
            self.plotData.contact_pos(:,k);
        scatter3(self.plotData.contact_pos(1,k),...
            self.plotData.contact_pos(2,k),...
            self.plotData.contact_pos(3,k),5,'filled','m');
        plot3([self.plotData.contact_pos(1,k),force_arrow_end(1)],...
            [self.plotData.contact_pos(2,k),force_arrow_end(2)],...
            [self.plotData.contact_pos(3,k),force_arrow_end(3)],...
            '-r','LineWidth',2);
        %   plot the wrist orientation
        R_wrist = quat2rotm(self.plotData.wrist_quat(:,k)');
        z_wrist = R_wrist(:,3);
        wrist_arrow_end = -10*z_wrist + self.plotData.contact_pos(:,k);
        plot3([self.plotData.contact_pos(1,k),wrist_arrow_end(1)],...
            [self.plotData.contact_pos(2,k),wrist_arrow_end(2)],...
            [self.plotData.contact_pos(3,k),wrist_arrow_end(3)],...
            '-g','LineWidth',2);
        frame = getframe;
        writeVideo(v,frame);
        msg = sprintf('%3.2f percent ... ',k/N_samples*100);
        fprintf([reverseStr, msg]);
        reverseStr = repmat(sprintf('\b'), 1, length(msg));
    end
end
fprintf(' [ok].');
close(v);
end
