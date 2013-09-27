#!/usr/bin/env python
import rospy
import tf2_ros
import visualization_msgs.msg
import geometry_msgs.msg
import math
from visualization_msgs.msg import Marker
from bzrlib import transform

RADIUS = 0.2
REV_RADIUS = 1.4
Z_REV_RADIUS = 0.6
FRAME_ID = "/sphere_frame"
WORLD_FRAME_ID = "/world_frame"

MIN_ALPHA = 0
MAX_ALPHA = 2*math.pi
DELTA_ALPHA = MAX_ALPHA/40.0

MIN_PHI = 0
MAX_PHI = 2*math.pi
DELTA_PHI = MAX_PHI/60.0

if __name__ == '__main__':
    
    # ros initialization
    pub = rospy.Publisher('marker_msg',visualization_msgs.msg.Marker)
    rospy.init_node('marker_publisher');
    
    # marker message
    marker_msg = Marker()
    marker_msg.id = 1
    marker_msg.header.frame_id = FRAME_ID
    marker_msg.type = marker_msg.SPHERE
    marker_msg.action = Marker.ADD
    marker_msg.scale.x = RADIUS
    marker_msg.scale.y = RADIUS
    marker_msg.scale.z = RADIUS
    marker_msg.color.a = 1.0
    marker_msg.color.r = 1.0
    marker_msg.color.g = 1.0
    marker_msg.color.b = 0.0
    marker_msg.pose.position.x = 0
    marker_msg.pose.position.y = 0
    marker_msg.pose.position.z = 0
    marker_msg.pose.orientation.w = 1
    
    # transform broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    
    
    # position vars
    t = geometry_msgs.msg.TransformStamped()
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1
    
    t.header.frame_id = WORLD_FRAME_ID
    t.child_frame_id = FRAME_ID
    alpha = MIN_ALPHA
    phi = MIN_PHI
    
    while not rospy.is_shutdown():
        
        rospy.sleep(0.1)
        t.transform.translation.x = math.cos(alpha) * REV_RADIUS
        t.transform.translation.y = math.sin(alpha) * REV_RADIUS
        t.transform.translation.z = math.sin(phi) * Z_REV_RADIUS 
        
        t.header.stamp = rospy.Time.now()        
        tf_broadcaster.sendTransform(t)
        
        marker_msg.header.stamp = t.header.stamp
        pub.publish(marker_msg)
        
        alpha = alpha + DELTA_ALPHA
        if alpha > MAX_ALPHA:
            alpha = MIN_ALPHA
            
        phi = phi + DELTA_PHI
        if phi > MAX_PHI:
            phi = MIN_PHI
        