"""
Author       : HANG Tao (BCSC-EPA1, XC-DX/PJ-W3-PMT) Tao.HANG@cn.bosch.com
Date         : 2024-08-19 23:23:52
LastEditors  : HANG Tao (BCSC-EPA1, XC-DX/PJ-W3-PMT) Tao.HANG@cn.bosch.com
LastEditTime : 2024-08-20 10:14:58
FilePath     : /DynamicROSTopicSubscriber/dynamic_rostopic_subscriber/subscriber.py
Description  : 

Copyright (c) 2024 by Robert Bosch GmbH, All Rights Reserved. 
"""

import logging
from functools import wraps

import rospy
from genpy.dynamic import generate_dynamic
from rospy import AnyMsg

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


# Singleton decorator to ensure only one instance of the class
def singleton(cls):
    instances = {}

    def get_instance(*args, **kwargs):
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]

    return get_instance


@singleton
class DynamicROSTopicSubscriber:
    """
    Dynamic ROS topic subscriber class
    Allows dynamic subscription to ROS topics and handles messages
    """

    def __init__(self, node_name="dynamic_ros_topic_subscriber"):
        """
        Initialize the subscriber
        :param node_name: ROS node name
        """
        self.subscribers = {}  # Dictionary to store topics and callback functions
        self.cached_messages = {}  # Cache for generated message types
        self.node_initialized = False  # Flag to check if ROS node is initialized
        self.node_name = node_name  # ROS node name

    def subscribe(self, topic, msg_type=AnyMsg):
        """
        Decorator for subscribing to a topic
        :param topic: ROS topic to subscribe to
        :param msg_type: Message type, default is AnyMsg
        """

        def decorator(func):
            @wraps(func)
            def wrapper(*args, **kwargs):
                return func(*args, **kwargs)

            try:
                if not self.node_initialized:
                    rospy.init_node(self.node_name, anonymous=True)
                    self.node_initialized = True

                if topic not in self.subscribers:
                    self.subscribers[topic] = wrapper
                    rospy.Subscriber(topic, msg_type, self.callback)
                    logger.info(f"Subscribed to topic: {topic}")
                else:
                    logger.info(f"Already subscribed to topic: {topic}")
            except rospy.ROSException as e:
                logger.error(
                    f"ROS Exception occurred while subscribing to {topic}: {e}"
                )
            except Exception as e:
                logger.exception(
                    f"Unexpected error occurred while subscribing to {topic}: {e}"
                )

            return wrapper

        return decorator

    def spin(self):
        """
        Start processing ROS messages
        """
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            logger.info("ROS node shut down")
        except Exception as e:
            logger.exception(f"Unexpected error occurred during spin: {e}")

    def callback(self, msg: AnyMsg):
        """
        Handle received ROS messages
        :param msg: Received ROS message
        """
        try:
            connection_header = msg._connection_header

            topic = connection_header.get("topic", "")
            md5sum = connection_header.get("md5sum", "")
            if md5sum in self.cached_messages:
                msg_instance = self.cached_messages[md5sum]
            else:
                # Generate dynamic message
                msg_type = connection_header.get("type", "")
                message_definition = connection_header.get("message_definition", "")
                msg_spec = generate_dynamic(msg_type, message_definition)

                for _, msg_cls in msg_spec.items():
                    self.cached_messages[msg_cls._md5sum] = msg_cls()

                msg_instance = self.cached_messages[md5sum]

            deserialized_msg = msg_instance.deserialize(msg._buff)
            self.subscribers[topic](deserialized_msg)
        except KeyError as e:
            logger.error(f"KeyError occurred while processing message: {e}")
        except AttributeError as e:
            logger.error(f"AttributeError occurred while processing message: {e}")
        except Exception as e:
            logger.exception(f"Unexpected error occurred while processing message: {e}")


# Create global instance
ros_subscriber = DynamicROSTopicSubscriber()
