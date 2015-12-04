#!/usr/bin/env python
from copy import copy

import rospy
from atf_recorder import BagfileWriter
from obstacle_distance.srv import GetObstacleDistance


class RecordObstacleDistance:
    def __init__(self, topic_prefix, config_file, robot_config_file, write_lock, bag_file):
        self.topic_prefix = topic_prefix
        self.test_config = config_file
        self.robot_config_file = robot_config_file

        resources_timer_frequency = 100.0  # Hz
        self.timer_interval = 1 / resources_timer_frequency

        self.testblock_list = self.create_testblock_list()
        self.requested_objects = {}
        self.res_pipeline = {}

        self.BfW = BagfileWriter(bag_file, write_lock)

        rospy.loginfo("Waiting for obstacle_distance node...")
        rospy.wait_for_service(self.robot_config_file["obstacle_distance"]["services"])
        self.obstacle_distance_server = rospy.ServiceProxy(self.robot_config_file["obstacle_distance"]["services"],
                                                           GetObstacleDistance)

        rospy.Timer(rospy.Duration.from_sec(self.timer_interval), self.collect_obstacle_distances)

    def trigger_callback(self, msg):
        # Process Trigger
        pass

    def collect_obstacle_distances(self, event):
        pipeline = copy(self.res_pipeline)
        if not len(pipeline) == 0:
            pass
            # self.BfW.write_to_bagfile(topic, msg, rospy.Time.from_sec(time.time()))

    def create_testblock_list(self):

        testblock_list = {}
        for testblock in self.test_config:
            try:
                self.test_config[testblock]["obstacle_distance"]
            except KeyError:
                continue
            else:
                for item in self.test_config[testblock]["obstacle_distance"]:
                    if len(item) == 1:
                        # Single link or link chain to all objects
                        try:
                            testblock_list[testblock]
                        except KeyError:
                            testblock_list.update({testblock: {"all": []}})
                        testblock_list[testblock]["all"].append(item[0])
                    elif len(item) == 2:
                        # Single link or link chain to one or more objects
                        for object_name in item[1]:
                            try:
                                testblock_list[testblock][object_name]
                            except KeyError:
                                testblock_list[testblock].update({object_name: []})
                            testblock_list[testblock][object_name].append(item[0])
        return testblock_list
