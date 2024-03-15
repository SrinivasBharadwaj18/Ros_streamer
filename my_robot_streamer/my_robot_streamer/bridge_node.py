import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import paho.mqtt.client as mqtt
import cv2
import yaml
import os
import base64

class RosMqttVideoBridge(Node):
    def __init__(self, mqtt_config_path):
        super().__init__('ros_mqtt_video_bridge')

        # Load MQTT configuration from YAML file
        with open(mqtt_config_path, 'r') as file:
            mqtt_config = yaml.safe_load(file)

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.callback,
            10  # Message queue size
        )

        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_disconnect = self.on_disconnect

        # Extract MQTT broker information from the loaded configuration
        mqtt_broker_address = mqtt_config['broker']['host']
        mqtt_broker_port = mqtt_config['broker']['port']
        mqtt_username = mqtt_config['broker']['username']
        mqtt_password = mqtt_config['broker']['password']

        self.mqtt_client.username_pw_set(mqtt_username, mqtt_password)
        self.mqtt_client.connect(mqtt_broker_address, mqtt_broker_port, 60)

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT broker with result code %s" % rc)
        print("Connected to MQTT broker with result code %s" % rc)

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().info("Disconnected from MQTT broker with result code %s" % rc)
        print("Disconnected from MQTT broker with result code %s" % rc)

    def callback(self, msg):
        try:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg,'bgr8')
            _, image_data = cv2.imencode('.jpg', cv_image)
            payload = base64.b64encode(image_data)

            mqtt_topic = "mqtt_video"

            self.mqtt_client.publish(mqtt_topic, payload, qos=0, retain=False)
            self.get_logger().info(f"Published video frame to MQTT")

        except Exception as e:
            self.get_logger().error(f"Error in callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)

    mqtt_config_path = '/home/nikhil/Desktop/ros2_ws/src/my_robot_streamer/config/ros_mqtt_config.yaml'
    if os.path.exists(mqtt_config_path):

        node = RosMqttVideoBridge(mqtt_config_path)

        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    else:
        print(f"{mqtt_config_path},file path doesnt exist")

if __name__ == '__main__':
    main()
