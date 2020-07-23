#!/usr/bin/env python
# -*- coding: utf-8 -*

from box_publisher.cfg import PolygonArraytoPolygonConfig
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import PolygonStamped
from jsk_recognition_msgs.msg import PolygonArray
from jsk_topic_tools import ConnectionBasedTransport
from jsk_topic_tools.log_utils import logerr_throttle
import rospy


class PolygonArraytoPolygon(ConnectionBasedTransport):
    def __init__(self):
        super(PolygonArraytoPolygon, self).__init__()
        self.pub = self.advertise("~output", PolygonStamped, queue_size=1)
        Server(PolygonArraytoPolygonConfig, self._config_callback)

    def subscribe(self):
        self.sub = rospy.Subscriber(
            "~input", PolygonArray, self._convert)

    def unsubscribe(self):
        self.sub.unregister()

    def _config_callback(self, config, level):
        self.index = config.index
        return config

    def _convert(self, msg):
        polygon_msg = PolygonStamped()
        polygon_msg.header = msg.header
        if self.index < 0:
            return
        elif self.index < len(msg.polygons):
            polygon_msg = msg.polygons[self.index]
            self.pub.publish(polygon_msg)
        else:
            logerr_throttle(10,
                            'Invalid index {} is specified'
                            'for polygon array whose size is {}'
                            .format(self.index, len(msg.polygons)))


if __name__ == '__main__':
    rospy.init_node('polygon_array_to_polygon')
    PolygonArraytoPolygon()
    rospy.spin()
