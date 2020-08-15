#!/usr/bin/env python

#    source_location_visualization.py
#
#	 Copyright 2020 Tobias Krueger
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Pose
from std_msgs.msg import ColorRGBA


class SourceLocationVisualizer:
    SOURCE_LOCATION_ID = 0
    SOURCE_LOCATION_DESC_ID =1

    source_location = Marker(
        type = Marker.CUBE,
        ns = "SignalSource",
        id = SOURCE_LOCATION_ID,
        lifetime = rospy.Duration(secs=20),
        scale = Vector3(0.1, 0.1, 0.1),
        color = ColorRGBA(0.8, 0.0, 0.8, 1.0)
    )
    source_location_description = Marker(
        type = Marker.TEXT_VIEW_FACING,
        ns = "SignalSource",
        id = SOURCE_LOCATION_DESC_ID,
        lifetime = rospy.Duration(secs=20),
        scale = Vector3(0.1, 0.1, 0.1),
        color = ColorRGBA(0.0, 0.0, 0.5, 1.0),
        text= "Signal Source Location"
    )
    pose = Pose()
    def visualize(self):
        self.source_location.header.stamp = rospy.get_rostime()
        self.source_location_description.header.stamp = rospy.get_rostime()
        self.markerPublisher.publish(self.source_location)
        self.markerPublisher.publish(self.source_location_description)

    def __init__(self, frame_id):
        self.source_location.header.frame_id = frame_id
        self.source_location_description.header.frame_id = frame_id
        rospy.init_node('source_location_visualization')
        self.markerPublisher= rospy.Publisher('source_location', Marker, queue_size=2)
        router_location = rospy.get_param("/access_point_location")
        self.pose.position.x = router_location['x']
        self.pose.position.y = router_location['y']
        self.pose.position.z = router_location['z']
        self.pose.orientation.w = 1
        rospy.loginfo("Pose z is %f" % self.pose.position.z)
        self.source_location.pose = self.pose
        self.source_location.action = Marker.ADD
        self.source_location_description.pose = self.pose
        self.source_location_description.action = Marker.ADD


if __name__ == '__main__':
    visualizer = SourceLocationVisualizer('map')
    while not rospy.is_shutdown():
        visualizer.visualize()
        rospy.sleep(rospy.Duration(secs=20))
