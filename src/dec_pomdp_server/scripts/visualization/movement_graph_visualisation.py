#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class MovementGraphVisualizer:
    NAMESPACE= "MovementGraph"
    NODE_LIST_ID = 0
    EDGE_LIST_ID = 1
    node_list = Marker(
        type=Marker.SPHERE_LIST,
        ns= NAMESPACE,
        id= NODE_LIST_ID,
        lifetime=rospy.Duration(0),
        scale=Vector3(0.2,0.2,0.2),
        color=ColorRGBA(0.0,0.0,1.0,1.0)
    )
    edge_list = Marker(
        type=Marker.LINE_LIST,
        ns=NAMESPACE,
        id=EDGE_LIST_ID,
        lifetime=rospy.Duration(0),
        scale=Vector3(0.05,0.0,0.0),
        color=ColorRGBA(0.0,0.0,1.0,1.0)
    )

    def visualize_nodes(self, nodes):
        for node in nodes:
            self.node_list.points.append(nodes[node])
        self.nodePublisher.publish(self.node_list)

    def visualze_allowed_moves(self, nodes, allowed_moves):
        for node in nodes:
            reachable_nodes = allowed_moves[node]
            for reachable_node in reachable_nodes:
                self.edge_list.points.append(nodes[node])
                self.edge_list.points.append(nodes[reachable_node])
        self.edgePublisher.publish(self.edge_list)

    def __init__(self, frame_id):
        self.node_list.header.frame_id = frame_id
        self.edge_list.header.frame_id = frame_id
        self.nodePublisher = rospy.Publisher('movement_graph_nodes', Marker, queue_size=1)
        self.edgePublisher = rospy.Publisher('movement_graph_edges', Marker, queue_size=1)

if __name__ == '__main__':
    rospy.init_node('graph_visualization', log_level=rospy.INFO)
    visualizer = MovementGraphVisualizer('map')

    #Read nodes
    node_file = open("/informatik2/students/home/6tkruege/bachelor/src/dec_pomdp_algorithm/config/locations.txt", "r")
    nodes = dict()
    for line in node_file.readlines():
        line_content = [float(x) for x in line.split(" ")]
        nodes[int(line_content[0])] = Point(line_content[1],line_content[2], 0.0)
    node_file.close()

    #Read Allowed moves
    allowed_moves_file = open("/informatik2/students/home/6tkruege/bachelor/src/dec_pomdp_algorithm/config/allowed_moves.txt", "r")
    allowed_moves = dict()
    for line in allowed_moves_file.readlines():
        line_content = [int(x) for x in line.split(" ")]
        allowed_moves[line_content.pop(0)] = line_content
    allowed_moves_file.close()

    #Log nodes and moves
    for node in nodes:
        rospy.loginfo("Found node %d with position x=%f and y=%f", node, nodes[node].x, nodes[node].y)
    for key in allowed_moves:
        rospy.loginfo("allowed moves for node %d are %s", key, ''.join(str(x) for x in allowed_moves[key]))

    #visualize nodes and moves
    while not rospy.is_shutdown():
        visualizer.visualize_nodes(nodes)
        visualizer.visualze_allowed_moves(nodes, allowed_moves)
        rospy.sleep(rospy.Duration(secs=20))
