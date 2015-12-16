#!/usr/bin/env python
import time
from copy import copy

import rospy
from atf_msgs.msg import *
from atf_recorder import BagfileWriter
from obstacle_distance.srv import *


class RecordObstacleDistance:
    def __init__(self, topic_prefix, config_file, robot_config_file, write_lock, bag_file):
        self.topic = topic_prefix + "obstacle_distance"
        self.test_config = config_file
        self.robot_config_file = robot_config_file

        resources_timer_frequency = 100.0  # Hz
        self.timer_interval = 1 / resources_timer_frequency

        self.testblock_pipeline = []

        self.BfW = BagfileWriter(bag_file, write_lock)

        # Wait for obstacle_distance node
        rospy.loginfo(rospy.get_name() + ": Waiting for obstacle distance node...")
        rospy.wait_for_service(self.robot_config_file["obstacle_distance"]["services"][0])
        rospy.loginfo(rospy.get_name() + ": Obstacle distance node found!")
        self.obstacle_distance_server = rospy.ServiceProxy(self.robot_config_file["obstacle_distance"]["services"][0],
                                                           GetObstacleDistance)

        rospy.Timer(rospy.Duration.from_sec(self.timer_interval), self.collect_obstacle_distances)

    def trigger_callback(self, msg):

        # Only save data if testblock is in test_config.yaml
        if msg.name in self.test_config:
            self.update_requested_testblocks(msg)

        if msg.trigger.trigger == Trigger.ERROR:
            self.testblock_pipeline = []

    def update_requested_testblocks(self, msg):

        if msg.trigger.trigger == Trigger.ACTIVATE and "obstacle_distance" in self.test_config[msg.name]:
            self.testblock_pipeline.append(msg.name)

        elif msg.trigger.trigger == Trigger.FINISH and msg.name in self.testblock_pipeline:
            self.testblock_pipeline.remove(msg.name)

    def collect_obstacle_distances(self, event):
        pipeline = copy(self.testblock_pipeline)
        if len(pipeline) == 0:
            return
        links = {}
        msg = ObstacleDistance()

        for testblock in pipeline:
            msg_link = ObstacleDistanceLink()
            for item in self.test_config[testblock]["obstacle_distance"]:
                if len(item) == 1:
                    co_object = []
                else:
                    co_object = item[1]
                if isinstance(item[0], list):
                    link = item[0]
                else:
                    link = [item[0]]

                request = GetObstacleDistanceRequest(link, co_object)
                response = self.obstacle_distance_server(request)

                for idx, name in enumerate(response.link_to_object):
                    link_name = name.split("_to_")[0]
                    object_name = name.split("_to_")[1]
                    try:
                        links[link_name]
                    except KeyError:
                        links[link_name] = {}
                    links[link_name][object_name] = response.distances[idx]

            for name in links:
                msg_link.name = name
                for co_object_name in links[name]:
                    msg_link.objects.append(co_object_name)
                    msg_link.distances.append(links[name][co_object_name])

            msg.links.append(msg_link)

        self.BfW.write_to_bagfile(self.topic, msg, rospy.Time.from_sec(time.time()))
