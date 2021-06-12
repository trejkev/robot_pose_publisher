#! /usr/bin/env python

__author__      = "Kevin Trejos Vargas"
__email__       = "kevin.trejosvargas@ucr.ac.cr"

"""
    Description:
        Robot ground truth pose publisher when simulating through Gazebo. It takes
        the robotModelName and publish its pose through /robotPosePublisher topic,
        referenced to referenceModelName, at a rate defined by samplingRate.
"""

import rospy                                                                                  # Needed for ros and python interaction
from std_msgs.msg import Header                                                               # Needed for message header
from geometry_msgs.msg import PoseStamped                                                     # Needed to publish the bot pose in an evo-allowed format
from gazebo_msgs.srv import GetModelState                                                     # Needed to use the gazebo service to get the model pose

#Global variables
samplingRate       = 150                                                                      # Samples per second to publish the robot pose
robotModelName     = 'turtlebot3_burger'                                                      # Robot model name
referenceModelName = 'ground_plane'                                                           # Reference model name

# Tailored message to publish the robot pose
message         = PoseStamped()
header          = Header()
header.frame_id = '/turtlebot3_burger'

rospy.init_node('robotPosePublisher')                                                         # Robot pose publisher node creation
robotPosePublisher = rospy.Publisher('/robotPosePublisher', PoseStamped, queue_size = 10)     # Robot pose publisher topic name, message type, and queue size statements
rospy.wait_for_service('/gazebo/get_model_state')                                             # Waiting for gazebo service availability
actualState = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)                    # Opening gazebo service for getting the robot state

rate = rospy.Rate(samplingRate)                                                               # Samples per second to retrieve the robot pose

while not rospy.is_shutdown():
    result = actualState(robotModelName, referenceModelName)                                  # Get the robotModelName state referenced to referenceModelName

    # Saving into message the robot pose to publish
    message.pose   = result.pose
    header.stamp   = rospy.Time.now()
    message.header = header

    robotPosePublisher.publish (message)                                                      # Publish on the topic the robot pose

    rate.sleep()
