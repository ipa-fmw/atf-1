#!/usr/bin/env python
import time

import rospy
from actionlib import SimpleActionClient
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
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """

        return CheckInterface(params)


class CheckInterface:
    def __init__(self, parameter):
        self.parameter = parameter
        self.finished = False
        self.results = {}

        rospy.Subscriber("/atf/interface", TestblockInterface, self.check_interface)

    def start(self):
        # Check for publisher
        for publisher in self.parameter["publisher"]:
            try:
                rospy.wait_for_message(publisher["topic"], publisher["type"], timeout=self.timeout)
            except rospy.ROSException:
                self.results["publisher"].update({publisher["topic"]: 0})
            else:
                self.results["publisher"].update({publisher["topic"]: 1})

        # Check for subscriber
        for subscriber in self.parameter["subscriber"]:
            pub = rospy.Publisher(subscriber["topic"], subscriber["type"])
            time_start = time.time()
            while (time.time() - time_start) < self.timeout:
                if pub.get_num_connections() > 0:
                    self.results["subscriber"].update({subscriber["topic"]: 1})
                    break
                else:
                    self.results["publisher"].update({subscriber["topic"]: 0})

        # Check for services
        for service in self.parameter["services"]:
            try:
                rospy.wait_for_service(service, timeout=self.timeout)
            except rospy.ROSException:
                self.results["services"].update({service: 0})
            else:
                self.results["services"].update({service: 1})

        # Check for actions
        for action in self.parameter["actions"]:
            client = SimpleActionClient(action["topic"], action["type"])

            if client.wait_for_server(rospy.Duration(self.timeout)):
                self.results["actions"].update({action["topic"]: 1})
            else:
                self.results["actions"].update({action["topic"]: 0})

    def stop(self):
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
