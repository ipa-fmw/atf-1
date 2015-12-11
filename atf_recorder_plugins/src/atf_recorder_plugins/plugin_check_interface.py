#!/usr/bin/env python
from copy import copy

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

        if msg.trigger.trigger == Trigger.ACTIVATE:
            self.testblock_pipeline.append(msg.name)

        elif msg.trigger.trigger == Trigger.FINISH:
            self.testblock_pipeline.remove(msg.name)

    def check_interfaces(self, event):
        pipeline = copy(self.testblock_pipeline)
        msg = TestblockInterface()

        for testblock in pipeline:
            testblock_interface = TestblockInterfaceStatus()
            testblock_interface.name = testblock
            interface = self.test_config[testblock]["interface"]
            timeout = interface["timeout"]

            for publisher in interface["publisher"]:
                testblock_interface.publisher.append(publisher["topic"])
                try:
                    rospy.wait_for_message(publisher["topic"], publisher["type"], timeout=timeout)
                except rospy.ROSException:
                    testblock_interface.publisher_results.append(False)
                else:
                    testblock_interface.publisher_results.append(True)

            # Check for subscriber
            for subscriber in interface["subscriber"]:
                testblock_interface.publisher.append(subscriber["topic"])
                pub = rospy.Publisher(subscriber["topic"], subscriber["type"])
                rospy.sleep(timeout)
                if pub.get_num_connections() > 0:
                    testblock_interface.subscriber_results.append(True)
                else:
                    testblock_interface.subscriber_results.append(False)

            # Check for services
            for service in interface["services"]:
                try:
                    rospy.wait_for_service(service, timeout=timeout)
                except rospy.ROSException:
                    testblock_interface.services_results.append(False)
                else:
                    testblock_interface.services_results.append(True)

            # Check for actions
            for action in interface["actions"]:
                client = SimpleActionClient(action["topic"], action["type"])

                if client.wait_for_server(rospy.Duration(timeout)):
                    testblock_interface.actions_results.append(True)
                else:
                    testblock_interface.actions_results.append(False)

            msg.testblock.append(testblock_interface)

        self.BfW.write_to_bagfile(self.topic, msg, rospy.Time.from_sec(time.time()))
