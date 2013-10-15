#!/usr/bin/env python
import rospy
import roslib.packages
import sys
import os
import subprocess
import time

if __name__=="__main__":
    
    # running subprocess with launch file
    proc_args = ['roslaunch','rosbridge_example','publish_image.launch']
#     proc_args = ['roslaunch rosbridge_example publish_image.launch']
    p = subprocess.Popen(proc_args)   
    print "started subprocess"
    
    
    # waiting for subprocess
    max_counter = 20
    counter = 0
    while counter < max_counter:
        
        time.sleep(1)
        counter = counter + 1;
        
        print "%i seconds to terminate subprocess" %(max_counter - counter)
        
    
    # terminate subprocess
    p.terminate()
    p.wait()
    
    print "subprocess finished"