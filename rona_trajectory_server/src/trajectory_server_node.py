#!/usr/bin/env python

#!/usr/bin/env python  
import rospy
import math
import tf
from nav_msgs.msg      import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg      import Float32
from std_srvs.srv      import Empty

import numpy

# todo: add threshold for new pose append to path
# todo: make frame ids variable
# todo: add metadata of path



def clearPath(req):
    rospy.loginfo("Clearing path and its metadata")

    global path
    global way_length


    del path.poses[:]
    way_length = 0.0

    return



if __name__ == '__main__':


    rospy.init_node('trajectory_server')

    # get parameters from launch file
    frame_id_map = rospy.get_param('~frame_id_map', 'map')
    rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~frame_id_map'),  frame_id_map)

    frame_id_pose = rospy.get_param('~frame_id_pose', 'laser')
    rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~frame_id_pose'), frame_id_pose)

    path_topic = rospy.get_param('~path_topic', 'trajectory')
    rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~path_topic'),    path_topic)

    loop_rate = rospy.get_param('~loop_rate', 10)
    rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~loop_rate'),     loop_rate)

    global path
    global way_length

    way_length = 0.0

    path =   Path()

    prefix = "/rona/"

    path_pub        = rospy.Publisher(prefix + path_topic, Path, queue_size=10)
    path_length_pub = rospy.Publisher(prefix + path_topic + "/length", Float32, queue_size=10)
    clear_path_srv  = rospy.Service(  prefix + path_topic + "/clear",  Empty, clearPath)

    # initialize path 
    path.header.frame_id = frame_id_map

    rate = rospy.Rate(loop_rate)
    rospy.loginfo('Finished initialization of trajectory server')

    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(frame_id_map, frame_id_pose, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # generate stamped pose from tf
        stampedPose                     = PoseStamped()
        stampedPose.header.stamp        = rospy.Time.now()
        stampedPose.header.frame_id     = frame_id_pose

        stampedPose.pose.position.x     = trans[0] 
        stampedPose.pose.position.y     = trans[1]
        stampedPose.pose.position.z     = trans[2]
        stampedPose.pose.orientation.x  = rot[0]
        stampedPose.pose.orientation.y  = rot[1]
        stampedPose.pose.orientation.z  = rot[2]
        stampedPose.pose.orientation.w  = rot[3]

        rospy.logdebug("Current translation: " + str(trans))
        rospy.logdebug("Current rotation:    " + str(rot))

        path.poses.append(stampedPose)


        rospy.logdebug("Number of waypoints  " + str(len(path.poses)))


        # calculate driven way
        old_pose   = Pose(); 

        # calculate length recursive
        if(len(path.poses) < 2):
            continue
    
        x_delta     = path.poses[-1].pose.position.x - path.poses[-2].pose.position.x
        y_delta     = path.poses[-1].pose.position.y - path.poses[-2].pose.position.y

        x_square    = math.pow(x_delta, 2)
        y_square    = math.pow(y_delta, 2)

        way_length += math.sqrt(x_square + y_square)


        path_length_msg = Float32(way_length)
        path_length_pub.publish(path_length_msg)

        rospy.logdebug("Length of trajectory: " + str(way_length))

        path.header.stamp    = rospy.Time.now()         # update time stamp for path
        path_pub.publish(path)                          # publish path

        rospy.logdebug('published path')
        rate.sleep()