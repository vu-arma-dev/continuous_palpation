function publisher=init_trajectory()
    msgtype = 'geometry_msgs/PoseArray';
    publisher=rospublisher('/set_continuous_palpation_trajectory', msgtype);
end