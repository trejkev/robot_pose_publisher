#! /usr/bin/env python

__author__      = "Kevin Trejos Vargas"
__email__       = "kevin.trejosvargas@ucr.ac.cr"

import rospy                                                                    # Needed for ros and python interaction
from std_msgs.msg      import Header                                            # Needed to publish the bot message header
from geometry_msgs.msg import PoseStamped                                       # Needed to publish the bot pose data
from gazebo_msgs.srv   import GetModelState                                     # Needed to use gazebo service to get the model pose

rospy.init_node('robotGTPosePublisher')  

# -- Tailored message to publish the robot pose
message = PoseStamped()
header  = Header()

################################################################################
############################ -- ASSIGNING PARAMS -- ############################
################################################################################

samplingRate         = 20
header.frame_id      = 'turtlebot3_burger'
robotModelName       = 'turtlebot3_burger'
publisher_topic_name = "robotGTPosePublisher"
referenceModelName   = 'ground_plane'

if rospy.has_param("~sampling_rate"):
    samplingRate = rospy.get_param("~sampling_rate")
if rospy.has_param("~robot_name"):
    header.frame_id = robotModelName = rospy.get_param("~robot_name")
if rospy.has_param("~publisher_topic_name"):
    publisher_topic_name = rospy.get_param("~publisher_topic_name")
if rospy.has_param("~reference_plane"):
    referenceModelName = rospy.get_param("~reference_plane")

################################################################################
############################# -- LAUNCHING TOPIC -- ############################
################################################################################

Publisher = rospy.Publisher(publisher_topic_name, PoseStamped, queue_size = 10)
rate = rospy.Rate(samplingRate)                                                 # Samples per second to get the robot pose
counter = 0
while not rospy.is_shutdown():
    rospy.wait_for_service('/gazebo/get_model_state')                           # Waiting for gazebo service availability
    actualState = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)  # Opening gazebo service for getting the robot state
    try:
        result = actualState(robotModelName, referenceModelName)                # Get the robotModelName state referenced to referenceModelName
        message.pose   = result.pose
        header.stamp   = rospy.Time.now()
        message.header = header
        Publisher.publish(message)                                              # Publish on the topic the robot pose
        counter += 1
        rospy.loginfo("Published {} to {} model state, for the {} time".format(robotModelName, referenceModelName, counter))
    except:
        rospy.loginfo("Failed to get the actual state for robot {}".format(robotModelName))
        pass
    rate.sleep()
