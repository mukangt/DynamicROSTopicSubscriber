"""
Author       : HANG Tao (BCSC-EPA1, XC-DX/PJ-W3-PMT) Tao.HANG@cn.bosch.com
Date         : 2024-08-19 23:23:52
LastEditors  : HANG Tao (BCSC-EPA1, XC-DX/PJ-W3-PMT) Tao.HANG@cn.bosch.com
LastEditTime : 2024-08-19 23:50:15
FilePath     : /DynamicROSTopicSubscriber/dynamic_rostopic_subscriber/subscriber.py
Description  : 

Copyright (c) 2024 by Robert Bosch GmbH, All Rights Reserved. 
"""

from functools import wraps

import rospy
from genpy.dynamic import generate_dynamic
from rospy import AnyMsg


# 单例装饰器，确保类只有一个实例
def singleton(cls):
    instances = {}

    def get_instance(*args, **kwargs):
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]

    return get_instance


@singleton
class DynamicTopicSubscriber:
    """
    动态 ROS 主题订阅器类
    允许动态订阅 ROS 主题并处理消息
    """

    def __init__(self, node_name="dynamic_topic_subscriber"):
        """
        初始化订阅器
        :param node_name: ROS 节点名称
        """
        self.subscribers = {}  # 存储主题和回调函数的字典
        self.cached_messages = {}  # 缓存已生成的消息类型
        self.node_initialized = False  # 标记 ROS 节点是否已初始化
        self.node_name = node_name  # ROS 节点名称

    def subscribe(self, topic, msg_type=AnyMsg):
        """
        订阅主题的装饰器
        :param topic: 要订阅的 ROS 主题
        :param msg_type: 消息类型，默认为 AnyMsg
        """

        def decorator(func):
            @wraps(func)
            def wrapper(*args, **kwargs):
                return func(*args, **kwargs)

            if not self.node_initialized:
                rospy.init_node(self.node_name, anonymous=True)
                self.node_initialized = True

            if topic not in self.subscribers:
                self.subscribers[topic] = wrapper
                rospy.Subscriber(topic, msg_type, self.callback)
                print(f"Subscribed to topic: {topic}")
            else:
                print(f"Already subscribed to topic: {topic}")

            return wrapper

        return decorator

    def spin(self):
        """
        开始处理 ROS 消息
        """
        rospy.spin()

    def callback(self, msg: AnyMsg):
        """
        处理接收到的 ROS 消息
        :param msg: 接收到的 ROS 消息
        """
        connection_header = msg._connection_header

        topic = connection_header.get("topic", "")
        md5sum = connection_header.get("md5sum", "")
        if md5sum in self.cached_messages:
            msg_instance = self.cached_messages[md5sum]
        else:
            # 生成动态消息
            msg_type = connection_header.get("type", "")
            message_definition = connection_header.get("message_definition", "")
            msg_spec = generate_dynamic(msg_type, message_definition)

            for _, msg_cls in msg_spec.items():
                self.cached_messages[msg_cls._md5sum] = msg_cls()

            msg_instance = self.cached_messages[md5sum]

        deserialized_msg = msg_instance.deserialize(msg._buff)
        self.subscribers[topic](deserialized_msg)


# 创建全局实例
ros_subscriber = DynamicTopicSubscriber()
