#!/usr/bin/env python

import rospy
from hera_ros_template.msg import ExampleMessage
from hera_ros_template.srv import ExampleService, ExampleServiceResponse

class ROSNodeTemplate:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('ros_node_template')

        # Publisher
        self.publisher = rospy.Publisher('example_message_topic', ExampleMessage, queue_size=10)

        # Subscriber
        rospy.Subscriber('example_message_input', ExampleMessage, self.callback)

        # Service
        self.service = rospy.Service('example_service', ExampleService, self.handle_service)

        # Rate for the main loop
        self.rate = rospy.Rate(1)  # 1 Hz

    def callback(self, data):
        # Subscriber callback function
        rospy.loginfo(f'Received message: id={data.id}, name={data.name}, value={data.value}')

    def handle_service(self, req):
        # Service callback function
        response = ExampleServiceResponse()
        response.success = True
        response.message = f'Received in service: id={req.request_data.id}, name={req.request_data.name}, value={req.request_data.value}'
        return response

    def publish_example_message(self):
        # Publishes an ExampleMessage
        message = ExampleMessage()
        message.id = 1
        message.name = "Test"
        message.value = 100.0
        self.publisher.publish(message)

    def run(self):
        # Main loop
        while not rospy.is_shutdown():
            self.publish_example_message()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ROSNodeTemplate()
        node.run()
    except rospy.ROSInterruptException:
        pass
