import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class SearchTagNode(Node):
    def __init__(self):
        super().__init__('search_tag_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10)
        self.cv_bridge = CvBridge()
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.target_tag_id = 42  # ID del tag da cercare
        self.desired_distance = 1.0  # Distanza desiderata dal tag
        self.k_linear = 0.1  # Costante di proporzionalità per il controllo lineare
        self.k_angular = 0.1  # Costante di proporzionalità per il controllo angolare

    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Rileva i tag ArUco nell'immagine utilizzando OpenCV
        tag_center, tag_orientation = self.detect_tag(cv_image)

        if tag_center is not None:
            # Calcola l'errore di distanza dal tag
            error_distance = self.desired_distance - tag_center[2]
            
            # Calcola il comando di velocità lineare proporzionale all'errore di distanza
            linear_x = self.k_linear * error_distance
            
            # Calcola il comando di velocità angolare proporzionale all'angolo di orientazione del tag
            angular_z = self.k_angular * tag_orientation[2]
            
            # Pubblica il comando di velocità
            self.publish_velocity(linear_x, angular_z)
        else:
            # Se il tag non è trovato, arresta il movimento
            self.publish_velocity(0.0, 0.0)

    def detect_tag(self, image):
        # Esegui il riconoscimento dei tag ArUco utilizzando OpenCV
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)
        
        if ids is not None and self.target_tag_id in ids:
            # Se il tag è trovato, calcola il centro e l'orientazione del tag
            index = np.where(ids == self.target_tag_id)[0][0]
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[index], 0.1, self.camera_matrix, self.dist_coeff)
            return tuple(tvec.squeeze()), tuple(rvec.squeeze())
        else:
            # Se il tag non è trovato, restituisci None
            return None, None

    def publish_velocity(self, linear_x, angular_z):
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.pub_cmd_vel.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    search_tag_node = SearchTagNode()
    rclpy.spin(search_tag_node)
    search_tag_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
