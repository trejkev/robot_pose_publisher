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

# -- Getting sampling rate from params
if rospy.has_param("~sampling_rate"):
    samplingRate = rospy.get_param("~sampling_rate")
else:
    samplingRate = 20
# -- Getting robot name from params
if rospy.has_param("~robot_name"):
    header.frame_id = robotModelName = rospy.get_param("~robot_name")
else:
    header.frame_id = robotModelName = 'turtlebot3_burger'  
# -- Getting publisher topic name from params
if rospy.has_param("~publisher_topic_name"):
    publisher_topic_name = rospy.get_param("~publisher_topic_name")
else:
    publisher_topic_name = "robotGTPosePublisher"
# -- Getting reference plane from params
if rospy.has_param("~reference_plane"):
    referenceModelName = rospy.get_param("~reference_plane")
else:
    referenceModelName = 'ground_plane'

################################################################################
############################# -- LAUNCHING TOPIC -- ############################
################################################################################

Publisher = rospy.Publisher(publisher_topic_name, PoseStamped, queue_size = 10)
rospy.wait_for_service('/gazebo/get_model_state')                               # Waiting for gazebo service availability
actualState = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)      # Opening gazebo service for getting the robot state
rate = rospy.Rate(samplingRate)                                                 # Samples per second to get the robot pose
while not rospy.is_shutdown():
    result = actualState(robotModelName, referenceModelName)                    # Get the robotModelName state referenced to referenceModelName
    message.pose   = result.pose
    header.stamp   = rospy.Time.now()
    message.header = header
    Publisher.publish(message)                                                  # Publish on the topic the robot pose
    rate.sleep()
