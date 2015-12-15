#!/usr/bin/env python

import rospy
from atf_msgs.msg import *


class CheckInterfaceParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        self.params = {}

    @staticmethod
    def parse_parameter(params):
        """
        Method that returns the metric class with the given parameter.
        :param params: Parameter
        """

        return CheckInterface(params)


class CheckInterface:
    def __init__(self, parameter):
        self.parameter = parameter
        self.active = False
        self.finished = False
        self.results = {}

        rospy.Subscriber("/atf/interface", TestblockInterface, self.check_interface)

    def check_interface(self, msg):
        if self.active:
            if "publisher" in self.parameter:
                for publisher in self.parameter["publisher"]:
                    for testblock in msg.testblock:
                        for idx, pub in enumerate(testblock.publisher):
                            if pub == publisher and pub not in self.results:
                                self.results[pub] = int(testblock.publisher_results[idx])

            if "subscriber" in self.parameter:
                for subscriber in self.parameter["subscriber"]:
                    for testblock in msg.testblock:
                        for idx, sub in enumerate(testblock.subscriber):
                            if sub == subscriber and sub not in self.results:
                                self.results[sub] = int(testblock.subscriber_results[idx])

            if "actions" in self.parameter:
                for action in self.parameter["actions"]:
                    for testblock in msg.testblock:
                        for idx, act in enumerate(testblock.actions):
                            if act == action and act not in self.results:
                                self.results[act] = int(testblock.actions_results[idx])

            if "services" in self.parameter:
                for service in self.parameter["services"]:
                    for testblock in msg.testblock:
                        for idx, serv in enumerate(testblock.services):
                            if serv == service and serv not in self.results:
                                self.results[serv] = int(testblock.services_results[idx])

    def start(self):
        self.active = True

    def stop(self):
        self.active = False
        self.finished = True

    @staticmethod
    def pause():
        pass

    @staticmethod
    def purge():
        pass

    def get_result(self):
        if self.finished:
            return "interface", self.results
        else:
            return False
