#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from dynamic_reconfigure.server import Server
from waterlinked_ros_driver.cfg import WaterlinkedParamsConfig


global max_points
max_points = 100

raw_points = Marker()
raw_points.header.frame_id = '/map'
raw_points.id = 0
raw_points.type = Marker.POINTS
raw_points.scale.x = 0.1
raw_points.scale.y = 0.1
raw_points.color.r = 1.0
raw_points.color.g = 0.0
raw_points.color.b = 0.0
raw_points.color.a = 1.0
raw_points.lifetime = rospy.Duration()

filtered_points = Marker()
filtered_points.header.frame_id = '/map'
filtered_points.id = 0
filtered_points.type = Marker.POINTS
filtered_points.scale.x = 0.1
filtered_points.scale.y = 0.1
filtered_points.color.r = 0.0
filtered_points.color.g = 0.0
filtered_points.color.b = 1.0
filtered_points.color.a = 1.0
filtered_points.lifetime = rospy.Duration()

wall = Marker()
wall.header.frame_id = '/map'
wall.id = 0
wall.type = Marker.LINE_LIST
wall.scale.x = 0.3
wall.color.r = 0.0
wall.color.g = 0.0
wall.color.b = 0.0
wall.color.a = 1.0
a, b, c, d = Point(), Point(), Point(), Point()
a.x, a.y, a.z = 0, 0, 0
b.x, b.y, b.z = 0, 20, 0
c.x, c.y, c.z = -4.88, 0, 0
d.x, d.y, d.z = -4.88, 20, 0
wall.points.append(a)
wall.points.append(b)
wall.points.append(c)
wall.points.append(d)
wall.lifetime = rospy.Duration()


def config_callback(config, level):
    global max_points
    max_points = config['Length']
    return config


def callback(msg, args):
    if args == 'raw':
        while len(raw_points.points) > max_points:
            raw_points.points.pop(0)
        raw_points.points.append(msg.point)
        raw_points.header.stamp = rospy.Time.now()
        raw_pub.publish(raw_points)
    else:
        while len(filtered_points.points) > max_points:
            filtered_points.points.pop(0)
        filtered_points.points.append(msg.point)
        filtered_points.header.stamp = rospy.Time.now()
        filtered_pub.publish(filtered_points)
    wall.header.stamp = rospy.Time.now()
    wall_pub.publish(wall)
    

if __name__ == '__main__':
    rospy.init_node('marker_pub')
    raw_sub = rospy.Subscriber('/waterlinked/acoustic_position/raw', PointStamped, callback, 'raw')
    filtered_sub = rospy.Subscriber('/waterlinked/acoustic_position/filtered', PointStamped, callback, 'filtered')
    raw_pub = rospy.Publisher('/waterlinked/acoustic_history/raw', Marker, queue_size=10)
    filtered_pub = rospy.Publisher('/waterlinked/acoustic_history/filtered', Marker, queue_size=10)
    wall_pub = rospy.Publisher('/waterlinked/acoustic_history/wall', Marker, queue_size=10)

    server = Server(WaterlinkedParamsConfig, config_callback)

    rospy.spin()
