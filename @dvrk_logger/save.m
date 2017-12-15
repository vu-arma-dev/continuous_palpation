function save(self, data_sub_folder_name)
%%  save the logger object
%   use this function only if you have setup the env variable
%   "PSMCMD"
if nargin<2
    data_sub_folder = [];
else
    data_sub_folder = [filesep,data_sub_folder_name];
end
root_path = self.datafolder;
data_path = [root_path,filesep,'Data',data_sub_folder];
if ~exist(data_path,'dir')
    mkdir(data_path);
end
logger = self;
save([data_path,filesep,self.logName],'logger');
end
