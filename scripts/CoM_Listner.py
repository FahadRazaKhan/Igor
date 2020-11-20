#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs
import tf

from persobalance import tool
import time
import numpy as np

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.point)
    # CoG_Position = data.point
    # print("CoG Position: ", CoG_Position)

    try:
        trans = tfBuffer.lookup_transform("map", "base_link", rospy.Time(0),rospy.Duration(0.01))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise
        
    
    CoG_transformed = tf2_geometry_msgs.do_transform_point(data, trans) # Transform Point from base_link frame to map frame
    #print("Transformed CoG: ", CoG_transformed)

    c = np.array([CoG_transformed.point.x, CoG_transformed.point.y, CoG_transformed.point.z])
    dcom, ac = com_sg.send( (time.time(), c) )

    # contact forces (f / total_mass)
    f = ac - gravity

    # zram
    alpha = (ground_level - c[2]) / f[2]

    zram = c + alpha * f

    #print("ZRAM: ", zram)
    print("CoG[2]: ", c[2])

    array = Float32MultiArray(data=zram)
    

    pub.publish(array)

    
    
def CoM_Listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    

    rospy.Subscriber("/cog/robot", PointStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    rospy.init_node('CoG_Listener', anonymous=True)

    pub = rospy.Publisher('ZRAM_chatter', Float32MultiArray, queue_size=10)

    # array=Float32MultiArray()
    # array.data.reshape(3)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    sg = 50
    degree = 2

    gravity = np.array([0, 0,-9.81])
    ground_level = 0.1016

    com_sg = tool.savitsky_golay(sg, degree, eval_at = 0, only_new = False)

    #pos_deriv = tool.savitsky_golay(sg, degree)

    CoM_Listener()