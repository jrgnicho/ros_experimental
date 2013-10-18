#!/usr/bin/env python

import rospy
import roslib
import Tkinter
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

class JogGui(Frame):
    
    def __init__(self,tk_root):
        
        Frame.__init__(self,tk_root)
        #self.frame_ = Frame(tk_root)
        self.tk_root_ = tk_root
        self.pack(side=Tkinter.TOP);
        
        self.images_ = []
        
    def create_layout(self):
        
        # x-y buttons 
        self.x_y_frame_ = Frame(self)
        
        # up button
        self.images_.append(PhotoImage(file=UP_ARROW_IMG_PATH))
        #self.y_pos_button_ = Button(self.x_y_frame_,justify=LEFT,image=self.images_[0],width=50,height=50)
        self.y_pos_button_ = Button(self.x_y_frame_,text='/\\')
        self.y_pos_button_.grid(row = 0,column = 1)
        
        # down button
        self.images_.append(PhotoImage(file=DOWN_ARROW_IMG_PATH))
        #self.y_neg_button_ = Button(self.x_y_frame_,justify=LEFT,image=self.images_[1],width=50,height=50)
        self.y_neg_button_ = Button(self.x_y_frame_,text='\\/')
        self.y_neg_button_.grid(row = 2,column = 1)
        
        # left button
        self.images_.append(PhotoImage(file=LEFT_ARROW_IMG_PATH))
        #self.x_neg_button_ = Button(self.x_y_frame_,justify=LEFT,image=self.images_[2],width=50,height=50)
        self.x_neg_button_ = Button(self.x_y_frame_,text='<')
        self.x_neg_button_.grid(row = 1 ,column = 0)
        
        # right button
        self.images_.append(PhotoImage(file=RIGHT_ARROW_IMG_PATH))
        #self.x_pos_button_ = Button(self.x_y_frame_,justify=LEFT,image=self.images_[2],width=50,height=50)
        self.x_pos_button_ = Button(self.x_y_frame_,text='>')
        self.x_pos_button_.grid(row = 1 ,column = 2)
        
        self.x_y_frame_.pack(side=TOP)
        
        return
        
    def run(self):
        
        self.create_layout()
        self.tk_root_.mainloop()
        return
        
        

if __name__ == "__main__":
    
    root = Tk()
    g = JogGui(root)
    g.run()
