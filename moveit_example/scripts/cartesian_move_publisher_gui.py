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
DELTA_X = 0.1 #meters
DELTA_Y = 0.1 #meters
DELTA_Z = 0.1 #meters

# widget size constants
IMAGE_WIDTH = 36
IMAGE_HEIGHT = 36

# topics
TRANSFORM_STAMPED_TOPIC = "tcp_delta_transform"

# frame id
WORLD_FRAME = "world_frame"

# parameters
PARAM_CONTINUOUS_PUBLISH = "allow_continuous_publishing"



class CartesianGuiPublisher(Frame):
    
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
        
        self.publish_new_transform_ = False
        self.allow_continuous_publishing_ = False
        
    def load_parameters(self):
        
        self.allow_continuous_publishing_ = rospy.get_param(PARAM_CONTINUOUS_PUBLISH, self.allow_continuous_publishing_)  
        return True      
        
    def enable_transform_publish(self,bool):
        self.publish_new_transform_ = bool
        
    def create_layout(self):
        
        # x-y buttons 
        self.x_y_frame_ = Frame(self,bd=2,relief = SUNKEN)
        
        # up button (y+)
        self.images_.append(self.open_image(UP_ARROW_IMG_PATH, IMAGE_WIDTH, IMAGE_HEIGHT))
        self.y_pos_button_ = Button(self.x_y_frame_,justify=LEFT,image=self.images_[0])
        #self.y_pos_button_ = Button(self.x_y_frame_,text='/\\')
        self.y_pos_button_.bind("<Button-1>",lambda event,arg = DELTA_Y:(self.enable_transform_publish(True),
                                                                         self.apply_translation(0, arg, 0)))
        self.y_pos_button_.bind("<ButtonRelease-1>", lambda event: (self.enable_transform_publish(True),
                                                                    self.apply_translation(0, 0, 0)))
        self.y_pos_button_.grid(row = 1,column = 2)
        
        # y plus label
        self.y_pos_label_ = Label(self.x_y_frame_,text="y+",justify=CENTER)
        self.y_pos_label_.grid(row = 0,column = 0,columnspan = 5)
        
        # down button (y-)
        self.images_.append(self.open_image(DOWN_ARROW_IMG_PATH,IMAGE_WIDTH,IMAGE_HEIGHT))
        self.y_neg_button_ = Button(self.x_y_frame_,justify=LEFT,image=self.images_[1])
        #self.y_neg_button_ = Button(self.x_y_frame_,text='\\/')
        self.y_neg_button_.bind("<Button-1>",lambda event,arg = -DELTA_Y: (self.enable_transform_publish(True),
                                                                           self.apply_translation(0, arg, 0)))
        self.y_neg_button_.bind("<ButtonRelease-1>", lambda event: (self.enable_transform_publish(True),
                                                                    self.apply_translation(0, 0, 0)))
        self.y_neg_button_.grid(row = 3,column = 2)
        
        # y minus label
        self.y_pos_label_ = Label(self.x_y_frame_,text="y-",justify=CENTER)
        self.y_pos_label_.grid(row = 4,column = 0,columnspan = 5)
        
        # left button (x-)
        self.images_.append(self.open_image(LEFT_ARROW_IMG_PATH,IMAGE_WIDTH,IMAGE_HEIGHT))
        self.x_neg_button_ = Button(self.x_y_frame_,justify=LEFT,image=self.images_[2])
        #self.x_neg_button_ = Button(self.x_y_frame_,text='<')
        self.x_neg_button_.bind("<Button-1>",lambda event,arg = -DELTA_X: (self.enable_transform_publish(True),
                                                                           self.apply_translation(arg, 0 , 0)))
        self.x_neg_button_.bind("<ButtonRelease-1>", lambda event: (self.enable_transform_publish(True),
                                                                    self.apply_translation(0, 0, 0)))
        self.x_neg_button_.grid(row = 2 ,column = 1)
        
        # x minus label
        self.x_neg_label_ = Label(self.x_y_frame_,text="x-",justify=CENTER)
        self.x_neg_label_.grid(row = 2,column = 0)
        
        # right button (x+)
        self.images_.append(self.open_image(RIGHT_ARROW_IMG_PATH,IMAGE_WIDTH,IMAGE_HEIGHT))
        self.x_pos_button_ = Button(self.x_y_frame_,justify=LEFT,image=self.images_[3])
        #self.x_pos_button_ = Button(self.x_y_frame_,text='>')
        self.x_pos_button_.bind("<Button-1>",lambda event,arg = DELTA_X: (self.enable_transform_publish(True),
                                                                          self.apply_translation(arg, 0 , 0)))
        self.x_pos_button_.bind("<ButtonRelease-1>", lambda event: (self.enable_transform_publish(True),
                                                                    self.apply_translation(0, 0, 0)))
        self.x_pos_button_.grid(row = 2 ,column = 3)
        
        # x plus label
        self.x_pos_label_ = Label(self.x_y_frame_,text="x+",justify=CENTER)
        self.x_pos_label_.grid(row = 2,column = 4)
        
        self.x_y_frame_.grid(row=0,column=0)
        
        # z buttons and labels frame
        self.z_frame_ = Frame(self,bd = 2 , relief = SUNKEN)
        
        # z up button
        self.images_.append(self.open_image(UP_ARROW_IMG_PATH, IMAGE_WIDTH, IMAGE_HEIGHT))        
        self.z_pos_button_ = Button(self.z_frame_,justify=LEFT,image=self.images_[4])
        self.z_pos_button_.bind("<Button-1>",lambda event,arg = DELTA_Z:(self.enable_transform_publish(True),
                                                                         self.apply_translation(0, 0, arg)))
        self.z_pos_button_.bind("<ButtonRelease-1>", lambda event: (self.enable_transform_publish(True),
                                                                    self.apply_translation(0, 0, 0)))
        self.z_pos_button_.grid(row = 1,column = 0)
        
        # z plus label
        self.z_pos_label_ = Label(self.z_frame_,text="z+",justify=CENTER)
        self.z_pos_label_.grid(row = 0,column = 0)
        
        # z down button
        self.images_.append(self.open_image(DOWN_ARROW_IMG_PATH, IMAGE_WIDTH, IMAGE_HEIGHT))        
        self.z_neg_button_ = Button(self.z_frame_,justify=LEFT,image=self.images_[5])
        self.z_neg_button_.bind("<Button-1>",lambda event,arg = -DELTA_Z:(self.enable_transform_publish(True),
                                                                         self.apply_translation(0, 0, arg)))
        self.z_neg_button_.bind("<ButtonRelease-1>", lambda event: (self.enable_transform_publish(True),
                                                                    self.apply_translation(0, 0, 0)))
        self.z_neg_button_.grid(row = 3,column = 0)
        
        # z minuz label
        self.z_pos_label_ = Label(self.z_frame_,text="z-",justify=CENTER)
        self.z_pos_label_.grid(row = 4,column = 0)
        
        # center blank label
        self.z_center_label_ = Label(self.z_frame_,text="\n"*2,justify=CENTER)
        self.z_center_label_.grid(row = 2,column = 0)
        
        self.z_frame_.grid(row=0,column = 1)
        
        
        
        
        
        return
    
    def publish_msg(self):    
        
        if (rospy.is_shutdown()):
            self.tk_root_.quit()
        
        if(self.publish_new_transform_):
            self.tf_stamped_publisher_.publish(self.transform_msg_)
            
        if(not self.allow_continuous_publishing_):  
            self.enable_transform_publish(False)
            
        self.after(100, self.publish_msg)
        
    
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
        
        self.load_parameters()
        self.create_layout()
        self.publish_msg()
        self.tk_root_.mainloop()
        return
        
        

if __name__ == "__main__":
    
    root = Tk()
    g = CartesianGuiPublisher(root)
    g.run()
