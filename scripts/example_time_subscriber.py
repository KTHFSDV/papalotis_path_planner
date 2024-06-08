#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

# This function is the callback that gets triggered whenever a new message is received on the 'current_time' topic
def callback(msg: String) -> None:
    """
    Callback function to log the received time from the 'current_time' topic.
    
    Args:
    - msg (String): Message received from the topic.
    """
    rospy.loginfo("Received time: %s", msg.data)

def main() -> None:
    """
    Main function to initialize the subscriber node and subscribe to the 'current_time' topic.
    """
    # Initializing the node with the name 'time_subscriber'
    rospy.init_node('time_subscriber')
    
    # Subscribing to the 'current_time' topic. Whenever a message is received on this topic, the callback function is triggered.
    rospy.Subscriber('current_time', String, callback)
    
    # Keeps the node running until manually terminated or a rospy-related exit signal is received.
    rospy.spin()

if __name__ == '__main__':
    main()
