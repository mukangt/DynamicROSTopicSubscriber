"""
Author       : HANG Tao (BCSC-EPA1, XC-DX/PJ-W3-PMT) Tao.HANG@cn.bosch.com
Date         : 2024-08-19 23:58:25
LastEditors  : HANG Tao (BCSC-EPA1, XC-DX/PJ-W3-PMT) Tao.HANG@cn.bosch.com
LastEditTime : 2024-08-19 23:58:27
FilePath     : /DynamicROSTopicSubscriber/tests/test_subscriber.py
Description  : 

Copyright (c) 2024 by Robert Bosch GmbH, All Rights Reserved. 
"""

import unittest
from unittest.mock import MagicMock, patch

import rospy
from genpy.dynamic import generate_dynamic
from subscriber import DynamicTopicSubscriber, ros_subscriber


class TestDynamicTopicSubscriber(unittest.TestCase):

    def setUp(self):
        self.subscriber = DynamicTopicSubscriber()

    @patch("rospy.init_node")
    @patch("rospy.Subscriber")
    def test_subscribe_decorator(self, mock_subscriber, mock_init_node):
        @self.subscriber.subscribe("/test_topic")
        def test_callback(msg):
            pass

        mock_init_node.assert_called_once_with(
            "dynamic_topic_subscriber", anonymous=True
        )
        mock_subscriber.assert_called_once_with(
            "/test_topic", rospy.AnyMsg, self.subscriber.callback
        )

    @patch("rospy.init_node")
    @patch("rospy.Subscriber")
    def test_subscribe_multiple_topics(self, mock_subscriber, mock_init_node):
        @self.subscriber.subscribe("/test_topic1")
        def test_callback1(msg):
            pass

        @self.subscriber.subscribe("/test_topic2")
        def test_callback2(msg):
            pass

        self.assertEqual(mock_subscriber.call_count, 2)
        mock_subscriber.assert_any_call(
            "/test_topic1", rospy.AnyMsg, self.subscriber.callback
        )
        mock_subscriber.assert_any_call(
            "/test_topic2", rospy.AnyMsg, self.subscriber.callback
        )

    @patch("rospy.spin")
    def test_spin(self, mock_spin):
        self.subscriber.spin()
        mock_spin.assert_called_once()

    @patch("genpy.dynamic.generate_dynamic")
    def test_callback(self, mock_generate_dynamic):
        mock_msg = MagicMock()
        mock_msg._connection_header = {
            "topic": "/test_topic",
            "md5sum": "test_md5sum",
            "type": "test_type",
            "message_definition": "test_definition",
        }
        mock_msg._buff = b"test_buffer"

        mock_msg_class = MagicMock()
        mock_msg_instance = MagicMock()
        mock_msg_class.return_value = mock_msg_instance
        mock_generate_dynamic.return_value = {"test_type": mock_msg_class}

        @self.subscriber.subscribe("/test_topic")
        def test_callback(msg):
            self.received_msg = msg

        self.subscriber.callback(mock_msg)

        mock_generate_dynamic.assert_called_once_with("test_type", "test_definition")
        mock_msg_instance.deserialize.assert_called_once_with(b"test_buffer")
        self.assertEqual(self.received_msg, mock_msg_instance.deserialize.return_value)

    def test_singleton(self):
        subscriber1 = DynamicTopicSubscriber()
        subscriber2 = DynamicTopicSubscriber()
        self.assertIs(subscriber1, subscriber2)
        self.assertIs(subscriber1, ros_subscriber)


if __name__ == "__main__":
    unittest.main()
