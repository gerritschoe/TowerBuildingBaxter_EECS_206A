#!/usr/bin/python

###############################################################################
##                   University of California, Berkeley                      ##
##---------------------------------------------------------------------------##
##                  EECS 206A | Introduction to Robotics                     ##
##                          Tower-Building Baxter                            ##
##---------------------------------------------------------------------------##
##          Team: Rob Ruigrok, Gerrit Schoettler and Ruhullah Najafi         ##
###############################################################################

import rospy
from ar_track_alvar_msgs.msg import *
from std_msgs.msg import Int16MultiArray, MultiArrayDimension, MultiArrayLayout


def callback(msg, prevMarkers):
    detectedMarkers = msg.markers
    # The current time in seconds
    now = rospy.get_time()
    for detectedMarker in detectedMarkers:
        measuredTime = detectedMarker.header.stamp.secs
        markerID = detectedMarker.id
        prevMarkers[markerID] = measuredTime

    detected_markers = list()
    for marker in prevMarkers.keys():
        if abs(prevMarkers[marker] - now) > 5:
            del prevMarkers[marker]
        else:
            detected_markers.append(marker)

    array = MultiArrayDimension()
    array.label = 'numMarkers'
    array.size = len(detected_markers)
    array.size = len(detected_markers)
    layout = MultiArrayLayout()
    layout.dim = [array,]
    layout.data_offset = 0

    msg = Int16MultiArray()
    msg.layout = layout
    msg.data = detected_markers
    pub.publish(msg)
    
if __name__ == '__main__':
    # This node is responsible for reading the ar_pose_marker topic and 
    # reporting back all markers that have been recently detected in the last 
    # five seconds.
    rospy.init_node('publishMarkersInfo')
    foundMarkers = {}
    global pub
    pub = rospy.Publisher(
        '/demo/found_markers', 
        Int16MultiArray, 
        queue_size = 10
    )
    rospy.Subscriber(
        "ar_pose_marker", 
        AlvarMarkers, 
        callback, 
        foundMarkers
    )

rospy.spin()