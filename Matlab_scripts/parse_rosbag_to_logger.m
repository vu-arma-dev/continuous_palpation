function parse_rosbag_to_logger(varargin)
%%  parse the rosbag to a dvrk_logger object and save it
rosbag_name = 'VU-18-02-20';
% All the rosbag files should be stored as
% [continuous_palpation/Data/rosbag/*.bag]
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'bag name')
            rosbag_name = propertyValue;
        end
    end
end
dvrk_init_continous_palp;
dataFolderPath = [getenv('CONT_PALP_DIR'),filesep,'Data'];
myTime=datetime;
logger = ...
    dvrk_logger([datestr(myTime,'mmm-dd_HH-MM-SS') '_Continuous'],...
    dataFolderPath);
logger.log_rosbag(...
    [dataFolderPath,filesep,'rosbag',filesep,rosbag_name,'.bag']);
logger.end_log;
logger.save;

end

