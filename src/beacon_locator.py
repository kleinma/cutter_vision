#!/usr/bin/env python                                                          
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Matthew Klein
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the names of the authors nor the names of their
# affiliated organizations may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseWithCovarianceStamped

import cv2

class beacon_locator(object):
  def __init__(self):
    #Init ROS
    rospy.init_node('beacon_locator')
    rospy.Subscriber('/camera/image', Image, self.imSubCB)
    self.statePub = rospy.Publisher('/cwru/visionStateEst', PoseWithCovarianceStamped, queue_size=1)
    
    #Query the paramter server to get the upper and lower HSV bounds for our beacon color
    self.lower_bound = rospy.get_param('~lower_bound',(  0,  0,  0))
    self.upper_bound = rospy.get_param('~upper_bound',(255,255,255))
    
    #Create state estimate message
    self.stateEst = PoseWithCovarianceStamped()
    #Create bridge to go between ROS messages and openCV images
    self.bridge = CvBridge()
    
  def imSubCB(self, msg):
    cvImage = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    self.stateEst.header.stamp = msg.header.stamp
    self.stateEst.header.frame_id = msg.header.frame_id
    # Do interesting thigns with image and esitmate state here.
    
    self.statePub.publish(self.stateEst)

  def run(self):
    rospy.spin()

if __name__ == "__main__":
  bl = beacon_locator()
  bl.run()
