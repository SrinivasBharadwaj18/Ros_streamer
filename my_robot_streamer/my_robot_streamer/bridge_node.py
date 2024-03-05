import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import paho.mqtt.client as mqtt
import cv2
import yaml

class RosMqttVideoBridge(Node):
    def __init__(self, mqtt_config_path):
        super().__init__('ros_mqtt_video_bridge')

        # Load MQTT configuration from YAML file
        with open(mqtt_config_path, 'r') as file:
            mqtt_config = yaml.safe_load(file)

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # Replace with your ROS image topic
            self.callback,
            10  # Message queue size
        )

        self.mqtt_client = mqtt.Client()
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

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().info("Disconnected from MQTT broker with result code %s" % rc)

    def callback(self, msg):
        try:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, image_data = cv2.imencode('.jpg', cv_image)
            payload = image_data.tobytes()

            mqtt_topic = "/ros/mqtt_video"  # Replace with your MQTT topic for video frames

            self.mqtt_client.publish(mqtt_topic, payload, qos=0, retain=False)
            self.get_logger().info(f"Published video frame to MQTT")

        except Exception as e:
            self.get_logger().error(f"Error in callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)

    # Provide the path to your MQTT configuration YAML file
    mqtt_config_path = '/home/nikhil/Desktop/ros2_ws/src/my_robot_streamer/config/ros_mqtt_config.yaml'

    node = RosMqttVideoBridge(mqtt_config_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
