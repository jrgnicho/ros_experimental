#!/usr/bin/env python

import rospy
import roslib
import Tkinter
import tf2_ros
import geometry_msgs.msg
from Tkinter import *
from Tkinter import Frame
from Tkinter import Button
from moveit_commander import MoveGroupCommander
from moveit_commander import RobotState

RESOURCES_PATH = roslib.packages.get_pkg_dir("moveit_example") + "/resources"
UP_ARROW_IMG_PATH = RESOURCES_PATH + "/up_blue.gif"
DOWN_ARROW_IMG_PATH = RESOURCES_PATH + "/down_blue.gif"
LEFT_ARROW_IMG_PATH = RESOURCES_PATH + "/left_blue.gif"
RIGHT_ARROW_IMG_PATH = RESOURCES_PATH + "/right_blue.gif"

# displacement constants
DELTA_X = 0.01 #meters
DELTA_Y = 0.01 #meters
DELTA_Z = 0.01 #meters

# topics
TRANSFORM_STAMPED_TOPIC = "tcp_delta_transform"

# frame id
WORLD_FRAME = "world_frame"

class JogGui(Frame):
    
    def __init__(self,tk_root):
        
        # tk initialization
        Frame.__init__(self,tk_root)
        #self.frame_ = Frame(tk_root)
        self.tk_root_ = tk_root
        self.pack(side=Tkinter.TOP);
        
        # images
        self.images_ = []
        
        # ros initialization
        rospy.init_node("moveit_tcp_trasform_publisher")
        
        # publisher
        self.tf_stamped_publisher_ = rospy.Publisher(TRANSFORM_STAMPED_TOPIC,geometry_msgs.msg.TransformStamped)
        
        # transform
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = WORLD_FRAME
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1
        self.transform_msg_ = t        
        
        
    def create_layout(self):
        
        # x-y buttons 
        self.x_y_frame_ = Frame(self)
        
        # up button
        self.images_.append(self.open_image(UP_ARROW_IMG_PATH, 36, 36))
        self.y_pos_button_ = Button(self.x_y_frame_,justify=LEFT,image=self.images_[0])
        #self.y_pos_button_ = Button(self.x_y_frame_,text='/\\')
        #self.y_pos_button_.configure(command = lambda arg = DELTA_Y: self.apply_translation(0, arg, 0))
        self.y_pos_button_.bind("<Button-1>",lambda event,arg = DELTA_Y: self.apply_translation(0, arg, 0))
        self.y_pos_button_.bind("<ButtonRelease-1>", lambda event,arg = 0.0: self.apply_translation(0, arg, 0))
        self.y_pos_button_.grid(row = 0,column = 1)
        
        # down button
        self.images_.append(self.open_image(DOWN_ARROW_IMG_PATH,36,36))
        self.y_neg_button_ = Button(self.x_y_frame_,justify=LEFT,image=self.images_[1])
        #self.y_neg_button_ = Button(self.x_y_frame_,text='\\/')
        #self.y_neg_button_.configure(command = lambda arg = -DELTA_Y: self.apply_translation(0, arg, 0))
        self.y_neg_button_.bind("<Button-1>",lambda event,arg = -DELTA_Y: self.apply_translation(0, arg, 0))
        self.y_neg_button_.bind("<ButtonRelease-1>", lambda event,arg = 0.0: self.apply_translation(0, arg, 0))
        self.y_neg_button_.grid(row = 2,column = 1)
        
        # left button
        self.images_.append(self.open_image(LEFT_ARROW_IMG_PATH,36,36))
        self.x_neg_button_ = Button(self.x_y_frame_,justify=LEFT,image=self.images_[2])
        #self.x_neg_button_ = Button(self.x_y_frame_,text='<')
        #self.x_neg_button_.configure(command = lambda arg = -DELTA_X: self.apply_translation(arg, 0 , 0))
        self.x_neg_button_.bind("<Button-1>",lambda event,arg = -DELTA_X: self.apply_translation(arg, 0 , 0))
        self.x_neg_button_.bind("<ButtonRelease-1>", lambda event,arg = 0.0: self.apply_translation(arg, 0 , 0))
        self.x_neg_button_.grid(row = 1 ,column = 0)
        
        # right button
        self.images_.append(self.open_image(RIGHT_ARROW_IMG_PATH,36,36))
        self.x_pos_button_ = Button(self.x_y_frame_,justify=LEFT,image=self.images_[3])
        #self.x_pos_button_ = Button(self.x_y_frame_,text='>')
        #self.x_pos_button_.configure(command = lambda arg = DELTA_X: self.apply_translation(arg, 0 , 0))
        self.x_pos_button_.bind("<Button-1>",lambda event,arg = DELTA_X: self.apply_translation(arg, 0 , 0))
        self.x_pos_button_.bind("<ButtonRelease-1>", lambda event,arg = 0.0: self.apply_translation(arg, 0 , 0))
        self.x_pos_button_.grid(row = 1 ,column = 2)
        
        self.x_y_frame_.pack(side=TOP)
        
        return
    
    def publish_msg(self):    
        
        if (rospy.is_shutdown()):
            self.tk_root_.quit()
        
        self.tf_stamped_publisher_.publish(self.transform_msg_)
        self.after(400, self.publish_msg)
        
    
    def apply_translation(self,dx = 0,dy=0,dz = 0):
        
        #print "delta = %f, %f, %f" %(dx,dy,dz)
        #self.transform_msg_ = geometry_msgs.msg.TransformStamped()
        self.transform_msg_.transform.translation.x = dx
        self.transform_msg_.transform.translation.y = dy
        self.transform_msg_.transform.translation.z = dz
        return
    
    def open_image(self,file_name,size_x = 10,size_y = 10):
        
        im = PhotoImage(file = file_name)
        #print "before resizing: width %i, height %i" %(im.width(),im.height())
        im = im.subsample(int(im.width()/size_x), int(im.height()/size_y))
        #print "after resizing: width %i, height %i" %(im.width(),im.height())
        return im
        
    def run(self):
        
        self.create_layout()
        self.publish_msg()
        self.tk_root_.mainloop()
        return
        
        

if __name__ == "__main__":
    
    root = Tk()
    g = JogGui(root)
    g.run()
