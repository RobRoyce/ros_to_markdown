#!/usr/bin/env python3
import time

import rospy
from std_srvs.srv import Trigger, TriggerResponse


def handle_delayed_request(req):
    rospy.loginfo("DelayedResponse: Received request. Sleeping 5 seconds...")
    time.sleep(5)
    return TriggerResponse(success=True, message="DelayedResponse done after 5s")


def main():
    rospy.init_node("delayed_response_server", anonymous=True)
    global service
    service = rospy.Service("/delayed_response", Trigger, handle_delayed_request)
    rospy.loginfo("DelayedResponse: service /delayed_response is ready.")
    rospy.spin()


if __name__ == "__main__":
    main()
