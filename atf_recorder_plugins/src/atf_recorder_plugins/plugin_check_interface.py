#!/usr/bin/env python
from copy import copy

import rostopic
from actionlib import *
from atf_msgs.msg import *
from atf_recorder import BagfileWriter


class CheckInterface:
    def __init__(self, topic_prefix, config_file, robot_config_file, write_lock, bag_file):
        self.topic = topic_prefix + "interface"
        self.test_config = config_file
        self.robot_config_file = robot_config_file

        resources_timer_frequency = 100.0  # Hz
        self.timer_interval = 1 / resources_timer_frequency
        self.default_timeout = 20  # s

        self.testblock_pipeline = []

        self.BfW = BagfileWriter(bag_file, write_lock)

        rospy.Timer(rospy.Duration.from_sec(self.timer_interval), self.check_interfaces)

    def trigger_callback(self, msg):

        # Only save data if testblock is in test_config.yaml
        if msg.name in self.test_config:
            self.update_requested_testblocks(msg)

        if msg.trigger.trigger == Trigger.ERROR:
            self.testblock_pipeline = []

    def update_requested_testblocks(self, msg):

        if msg.trigger.trigger == Trigger.ACTIVATE and "interface" in self.test_config[msg.name]:
            self.testblock_pipeline.append(msg.name)

        elif msg.trigger.trigger == Trigger.FINISH and msg.name in self.testblock_pipeline:
            self.testblock_pipeline.remove(msg.name)

    def check_interfaces(self, event):
        pipeline = copy(self.testblock_pipeline)
        if len(pipeline) == 0:
            return

        msg = TestblockInterface()
        for testblock in pipeline:
            testblock_interface = TestblockInterfaceStatus()
            interface = self.test_config[testblock]["interface"]

            if "timeout" in interface:
                timeout = interface["timeout"]
            else:
                timeout = self.default_timeout

            if "publisher" in interface:
                # Check for publisher
                for publisher in interface["publisher"]:
                    testblock_interface.publisher.append(publisher)
                    try:
                        rospy.wait_for_message(publisher, rostopic.get_topic_class(publisher, blocking=False)[0],
                                               timeout=timeout)
                    except (rospy.ROSException, rostopic.ROSTopicException):
                        testblock_interface.publisher_results.append(False)
                    else:
                        testblock_interface.publisher_results.append(True)

            if "subscriber" in interface:
                # Check for subscriber
                for subscriber in interface["subscriber"]:
                    testblock_interface.subscriber.append(subscriber["topic"])

                    pub = rospy.Publisher(subscriber["topic"], subscriber["class"])
                    rospy.sleep(timeout)

                    if pub.get_num_connections() > 0:
                        testblock_interface.subscriber_results.append(True)
                    else:
                        testblock_interface.subscriber_results.append(False)

            if "services" in interface:
                # Check for services
                for service in interface["services"]:
                    testblock_interface.services.append(service)
                    try:
                        rospy.wait_for_service(service, timeout=timeout)
                    except rospy.ROSException:
                        testblock_interface.services_results.append(False)
                    else:
                        testblock_interface.services_results.append(True)

            if "actions" in interface:
                # Check for actions
                for action in interface["actions"]:
                    testblock_interface.actions.append(action["topic"])
                    client = SimpleActionClient(action["topic"], action["class"])

                    if client.wait_for_server(rospy.Duration(timeout)):
                        testblock_interface.actions_results.append(True)
                    else:
                        testblock_interface.actions_results.append(False)

            testblock_interface.stamp = rospy.Time.from_sec(time.time())
            msg.testblock.append(testblock_interface)

        self.BfW.write_to_bagfile(self.topic, msg, rospy.Time.from_sec(time.time()))
