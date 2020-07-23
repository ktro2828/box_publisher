#!/usr/bin/env python
# -*- coding: utf-8 -*

import numpy as np
import rospy
from dynamic_reconfigure.server import Server
from jsk_recognition_msgs.msg import BoundingBox, PolygonArray

from box_publisher.cfg import DynamicParamsConfig


class BoxPublisher(object):
    def __init__(self):
        self.srv = Server(DynamicParamsConfig,
                          self.dynamic_reconfigure_callback)
        self.sub = rospy.Subscriber(
            "~input", PolygonArray, self.polygon_callback)
        self.box = BoundingBox()
        self.frame = rospy.get_param("~frame", "/base_link")
        self.box.header.frame_id = self.frame
        self.pub = rospy.Publisher("~output", BoundingBox, queue_size=1)

    def dynamic_reconfigure_callback(self, config, level):
        """Box's default pose
        """
        self.x = config["x"]
        self.y = config["y"]
        self.z = config["z"]
        self.w = config["w"]
        self.d = config["d"]
        self.h = config["h"]
        return config

    def polygon_callback(self, polygon_msg):
        if len(polygon_msg.polygons) > 1:
            rospy.loginfo("pub box")
            polygon_array_list = polygon_msg.polygons[1]
            rospy.loginfo(polygon_array_list)
            rospy.loginfo(polygon_array_list.polygon)
            rospy.loginfo(polygon_array_list.polygon.points)
            polygon_array = np.array(polygon_array_list)
            x_min, y_min, z_min = np.unravel_index(
                np.argmin(polygon_array), polygon_array.shape)
            x_max, y_max, z_max = np.unravel_index(
                np.argmax(polygon_array), polygon_array.shape)
            w = abs(x_min - x_max)
            d = abs(y_min - y_max)
            h = abs(z_min - z_max)
            x = (x_min + x_max) / 2
            y = (y_min + y_max) / 2
            z = (z_min + z_max) / 2
            self.box.header.stamp = rospy.Time.now()
            self.box.pose.position.x = x
            self.box.pose.position.y = y
            self.box.pose.position.z = z
            self.box.dimensions.x = w
            self.box.dimentions.y = d
            self.box.dimentions.z = h
            self.box.pose.orientation.w = 1
            self.box.pose.orientation.x = 0
            self.box.pose.orientation.y = 0
            self.box.pose.orientation.z = 0
            self.pub.publish(self.box)
        else:
            rospy.loginfo("TABLE is NONE")


def main():
    try:
        rospy.init_node("box_publisher", anonymous=False)
        bp = BoxPublisher()
        rospy.spin()
    except Exception:
        pass


if __name__ == '__main__':
    main()
