#!/usr/bin/env python
import rospy
import roslib
import md5
import random
import datetime
from std_msgs.msg import String

class MD5HashGenerator:
    
    def __init__(self):
        
        random.seed(datetime.datetime.now().second)
        self.random_list_ = [i for i in range(64,91)] + [i for i in range(97,123)] + [i for i in range(48,58)]
        
        char_list = []
        for i in self.random_list_:            
            char_list.append(chr(i))
            
        self.char_choices_ = ''.join(char_list)
        
        
    def generate_random_password(self,num_chars):
        
        password = []
        for i in range(0,num_chars):
            password.append(self.get_random_character())        
            
        return ''.join(password);
    
    def get_random_character(self):        
        return random.choice(self.char_choices_)
    
    def get_md5_hash(self,password):
        
        return md5.new(password).hexdigest()
    
    def generate_password_hash_msg(self,num_chars):
        
        p = self.generate_random_password(num_chars)
        h = self.get_md5_hash(p)
        
        return "[ Password: %s , MD5 Hash: %s ]" %(p,h)
    
if __name__ == "__main__":
        
        # initializing ros
        rospy.init_node("md5_hash_string_publisher")
        
        # publisher
        pub = rospy.Publisher("md5_hash", String)
        
        # publish
        md5_gen = MD5HashGenerator()
        msg = String()
        while not rospy.is_shutdown():
            
            msg.data = md5_gen.generate_password_hash_msg(8)
            #msg.data = "[ Password : " + md5_gen.generate_random_password(8) + " ]"
            pub.publish(msg)
            rospy.sleep(0.5)
        
        