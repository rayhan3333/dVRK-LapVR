#!/usr/bin/env python3
import rospy
from ambf_client import Client
from ambf_msgs.msg import ObjectState, WorldState
import time
import math
import socket
import json

_client = Client()
_client.connect()
print(_client.get_obj_names())

cover = _client.get_obj_handle('/ambf/env/Cover11')

while not rospy.is_shutdown():
    cover.set_pos(0.015, .2776, .744)