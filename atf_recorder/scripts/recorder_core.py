#!/usr/bin/env python
import os
import rospkg
import time
import yaml
from threading import Lock

import atf_recorder_plugins
import rosbag
import rosparam
import rospy
import rostopic
from atf_msgs.msg import *
from atf_recorder import BagfileWriter
from atf_recorder.srv import *


class ATFRecorder:
    def __init__(self):

        self.bag_name = rosparam.get_param("/test_name")
        self.number_of_tests = rosparam.get_param("/number_of_tests")
        self.robot_config_file = self.load_data(rosparam.get_param("/robot_config"))

        if not os.path.exists(rosparam.get_param(rospy.get_name() + "/bagfile_output")):
            os.makedirs(rosparam.get_param(rospy.get_name() + "/bagfile_output"))

        self.topic = "/atf/"
        self.lock_write = Lock()
        self.bag = rosbag.Bag(rosparam.get_param(rospy.get_name() + "/bagfile_output") + self.bag_name + ".bag", 'w')
        self.test_config = self.load_data(rosparam.get_param(rospy.get_name() + "/test_config_file")
                                          )[rosparam.get_param("/test_config")]
        recorder_config = self.load_data(rospkg.RosPack().get_path("atf_recorder_plugins") +
                                         "/config/recorder_plugins.yaml")

        self.BfW = BagfileWriter(self.bag, self.lock_write)

        # Init metric recorder
        self.recorder_plugin_list = []
        for (key, value) in recorder_config.iteritems():
            self.recorder_plugin_list.append(getattr(atf_recorder_plugins, value)(self.topic,
                                                                                  self.test_config,
                                                                                  self.robot_config_file,
                                                                                  self.lock_write,
                                                                                  self.bag))

        self.topic_pipeline = []
        self.active_sections = []
        self.requested_topics = []
        self.testblock_list = self.create_testblock_list()

        for topic in self.get_topics():
            msg_type = rostopic.get_topic_class(topic, blocking=True)[0]
            rospy.Subscriber(topic, msg_type, self.global_topic_callback, queue_size=5, callback_args=topic)

        self.test_status_publisher = rospy.Publisher(self.topic + "test_status", TestStatus, queue_size=10)
        rospy.Service(self.topic + "recorder_command", RecorderCommand, self.command_callback)

        # Wait for subscriber
        num_subscriber = self.test_status_publisher.get_num_connections()
        while num_subscriber == 0:
            num_subscriber = self.test_status_publisher.get_num_connections()

        test_status = TestStatus()
        test_status.test_name = self.bag_name
        test_status.status_recording = 1
        test_status.status_analysing = 0
        test_status.total = self.number_of_tests

        self.test_status_publisher.publish(test_status)

        rospy.loginfo(rospy.get_name() + ": Node started!")

    def shutdown(self):
        self.lock_write.acquire()
        self.bag.close()
        self.lock_write.release()

        test_status = TestStatus()
        test_status.test_name = self.bag_name
        test_status.status_recording = 3
        test_status.status_analysing = 0
        test_status.total = self.number_of_tests

        self.test_status_publisher.publish(test_status)

    def create_testblock_list(self):
        testblock_list = {}
        for testblock in self.test_config:
            for metric in self.test_config[testblock]:
                if metric in self.robot_config_file:
                    try:
                        testblock_list[testblock]
                    except KeyError:
                        testblock_list[testblock] = self.robot_config_file[metric]["topics"]
                    else:
                        for topic in self.robot_config_file[metric]["topics"]:
                            testblock_list[testblock].append(topic)
                else:
                    try:
                        if "topics" in self.test_config[testblock][metric]:
                            try:
                                testblock_list[testblock]
                            except KeyError:
                                testblock_list[testblock] = self.test_config[testblock][metric]["topics"]
                            else:
                                for topic in self.test_config[testblock][metric]["topics"]:
                                    testblock_list[testblock].append(topic)
                    except TypeError:
                        pass

        return testblock_list

    def update_requested_topics(self, msg):

        if msg.trigger.trigger == Trigger.ACTIVATE:
            for topic in self.testblock_list[msg.name]:
                self.requested_topics.append(topic)
                if topic not in self.topic_pipeline:
                    self.topic_pipeline.append(topic)

        elif msg.trigger.trigger == Trigger.FINISH:
            for topic in self.testblock_list[msg.name]:
                self.requested_topics.remove(topic)
                if topic not in self.requested_topics:
                    self.topic_pipeline.remove(topic)

    def command_callback(self, msg):

        if (msg.trigger.trigger == Trigger.ACTIVATE and msg.name in self.active_sections) or \
                (msg.trigger.trigger == Trigger.FINISH and msg.name not in self.active_sections) or \
                msg.name not in self.test_config:
            return RecorderCommandResponse(False)

        # Only process message if testblock requests topics
        if msg.name in self.testblock_list:
            self.update_requested_topics(msg)

        # Send message to all recorder plugins
        for recorder_plugin in self.recorder_plugin_list:
            recorder_plugin.trigger_callback(msg)

        if msg.trigger.trigger == Trigger.ACTIVATE:
            self.active_sections.append(msg.name)
        elif msg.trigger.trigger == Trigger.FINISH:
            self.active_sections.remove(msg.name)
        elif msg.trigger.trigger == Trigger.ERROR:
            self.topic_pipeline = []

        self.BfW.write_to_bagfile(self.topic + msg.name + "/Trigger", Trigger(msg.trigger.trigger),
                                  rospy.Time.from_sec(time.time()))

        return RecorderCommandResponse(True)

    @staticmethod
    def load_data(filename):

        with open(filename, 'r') as stream:
            doc = yaml.load(stream)
        return doc

    def global_topic_callback(self, msg, name):
        if name in self.topic_pipeline:
            now = rospy.Time.from_sec(time.time())
            try:
                for item in msg.transforms:
                    item.header.stamp = now
            except AttributeError:
                pass

            self.BfW.write_to_bagfile(name, msg, now)

    def get_topics(self):
        topics = []

        for item in self.test_config:
            for metric in self.test_config[item]:
                if metric in self.robot_config_file:
                    # Get topics from robot_config.yaml
                    for topic in self.robot_config_file[metric]["topics"]:
                        if topic not in topics:
                            topics.append(topic)
                else:
                    try:
                        if "topics" in self.test_config[item][metric]:
                            # Get topics from test_config.yaml
                            for topic in self.test_config[item][metric]["topics"]:
                                if topic not in topics:
                                    topics.append(topic)
                    except TypeError:
                        pass
        return topics


if __name__ == "__main__":
    rospy.init_node('atf_recorder')
    atf = ATFRecorder()
    rospy.on_shutdown(atf.shutdown)
    rospy.spin()
