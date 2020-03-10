#!/usr/bin/env python
import rospy
import json

if __name__ == '__main__':
    rospy.init_node("location_reader")
    file_path = rospy.get_param('~file-path')
    try:
        with open(file_path, 'r') as json_file:
            data = json.load(json_file)
            rospy.logwarn("reading locations file...")
            for location in data['locations']:
                rospy.logwarn("Node number %d has x %f and y %f", location['id'], location['x'], location['y'])
                rospy.set_param('planner/locations/' + str(location['id']), {'x': location['x'], 'y': location['y']})
                for allowed_moves in data['allowed_moves']:
                    rospy.set_param('planner/allowed_moves/' + str(allowed_moves['id']), allowed_moves['allowed_destinations'])
        rospy.logwarn("Finished reading locations")
    except IOError as e:
        rospy.logerr("The File at the location %s could not be read", file_path)
        raise
