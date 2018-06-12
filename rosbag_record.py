#!/usr/bin/env python

# Rosbag recording script that allows starting and stopping recording from scriptline
# Is mostly just a hack to run the bash script for "rosbag record" and then to kill all nodes starting with "record_"
# Adapted from https://gist.github.com/marco-tranzatto/8be49b81b1ab371dcb5d4e350365398a
# Which in turn was inspired by responses at https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/

import rospy
import subprocess
import os
import signal
import ipdb

class RosbagRecord:

    def __init__(self,fileInput='',folderInput='',start=False):
        
        self.b_recording=False
        self.filename=fileInput
        self.foldername=folderInput

        # If the foldername is empty, use the working directory
        if not self.foldername:
            self.set_foldername(os.getcwd())

        # Start the base node for recording
        rospy.init_node('rosbag_record')

        # Make sure to close cleanly on shutdown
        rospy.on_shutdown(self.stop_recording)

        # Start recording
        if start:
           self.start_recording()

    def terminate_ros_node(self, s):
        # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)

    def set_filename(self,fileInput):
        #TODO: set checks on filename?
        self.filename=fileInput

    def set_foldername(self,folderInput):
        self.foldername=folderInput

    def start_recording(self):
        if (not self.b_recording) and os.path.exists(self.foldername):
            # Start recording
            if self.filename:
                command = "rosbag record -a -o " + self.filename
            else:
                command = "rosbag record -a"
            self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self.foldername,
                                      executable='/bin/bash')
        self.b_recording=True

    def stop_recording(self):
        if self.b_recording:
            rospy.loginfo(rospy.get_name() + ' stop recording.')
            self.terminate_ros_node("/record_")
        self.b_recording=False

#---------------------------------------------------
# Set up some basic functions with user interaction
# for testing class functionality
#---------------------------------------------------
user_options = ['Quit',
                'Filename', 
                'Foldername',
                'Start Recording',
                'Stop Recording',
                'Print Options',
                'Debug']   
def print_options():
    i = 0;
    print ''
    for str in user_options:
        print i, (' : ' + str)
        i = i+1
        pass
    pass

if __name__ == '__main__':
    try:
        # Create a class instance to do the recording, add options to stop recording/change filenames
        rosbag_record = RosbagRecord('testA','/home/arma/catkin_ws/src/data')

        opt = '111'
        print_options()
        while int(opt) != 0:
            opt = raw_input('\nEnter option : ')
            if opt is '0':
                quit()
            elif opt is '1':
                fileInput = raw_input('\nEnter filename: ')
                rosbag_record.set_filename(fileInput)
            elif opt is '2':
                folderInput = raw_input('\nEnter foldername: ')
                rosbag_record.set_foldername(folderInput)
            elif opt is '3':
                rosbag_record.start_recording()
            elif opt is '4':
                rosbag_record.stop_recording()
            elif opt is '5':
                print_options()
            elif opt is '6':
                ipdb.set_trace()
            pass
        pass
    except rospy.ROSInterruptException:
        pass
