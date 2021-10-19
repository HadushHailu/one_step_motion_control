#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def odomCallback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    rospy.loginfo("[odom] (%f,%f)", x,y)

def amclCallback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    rospy.loginfo("[amcl] (%f,%f)", x,y)


    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('odom_amcl_pose', anonymous=True)

    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amclCallback)
    rospy.Subscriber("/odom", Odometry, odomCallback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
