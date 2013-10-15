#!/usr/bin/env python
import rospy
import roslib.packages
import sys
import os
import subprocess
import time
from std_msgs.msg import String
from std_msgs.msg import Bool

# ros topics
SUBPROCESS_ARGUMENTS="subprocess_arguments"
SUBPROCESS_STATUS = "subprocess_status"

# constants
NO_SUBPROCESS = "none"
INIT_SUBPROCESS_ARGS = ['echo','No process running at this time']

# global variables

class SubprocessLauncher:
    
    def __init__(self):
        
        # ros setup
        rospy.init_node('subprocess_launcher')
        
        # publisher
        self.pub_ = rospy.Publisher(SUBPROCESS_STATUS,Bool)        
        self.subproc_state_msg_ = Bool(False)
               
        # subscriber
        self.subs_ = rospy.Subscriber(SUBPROCESS_ARGUMENTS,String,self.subs_callback)
        
        # subprocess
        self.subproc_args_ = INIT_SUBPROCESS_ARGS
        self.subproc_ = None  
        
    def __del__(self):
        
        self.stop_subprocess()
        
    def start_subprocess(self):
        
        # run default subprocess
        self.subproc_ = subprocess.Popen(self.subproc_args_)   
        self.subproc_state_msg_.data = True      
        
        rospy.loginfo("Started subprocess: " + ' '.join(self.subproc_args_))        
        
    def stop_subprocess(self):        
        
        self.subproc_state_msg_.data = False
        self.subproc_.terminate()
        self.subproc_.wait()
        
        rospy.loginfo("Stopped subprocess")
        return
        
    def run(self):     
        
        # start default subprocess
        self.start_subprocess()
                  
        # publish status        
        while not rospy.is_shutdown():            
            self.pub_.publish(self.subproc_state_msg_.data)
            rospy.sleep(0.4)
            
    def cleanup(self):
        
        self.stop_subprocess()
        
            
    def subs_callback(self,msg):
        
        if msg.data == NO_SUBPROCESS:
            
            self.stop_subprocess()
            
        else:
            
            self.subproc_args_ = msg.data.split()
            self.start_subprocess()       


if __name__=="__main__":
    
    launcher = SubprocessLauncher()
    launcher.run()
    launcher.cleanup()