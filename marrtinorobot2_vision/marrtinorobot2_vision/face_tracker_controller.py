import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class FaceRecognitionAndTrackingNode(Node):
    def __init__(self):
        super().__init__('face_recognition_and_tracking_node')

        # Carica il modello di deep learning per il riconoscimento facciale
        self.net = cv2.dnn.readNetFromCaffe(
            'deploy.prototxt',
            'res10_300x300_ssd_iter_140000.caffemodel'
        )

        # Inizializza il CvBridge
        self.bridge = CvBridge()

        # Sottoscrizione al topic della fotocamera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher per il controllo del Dynamixel
        self.dynamixel_control = self.create_publisher(Float64, '/pan_controller/command', 10)
        self.dynamixel_control_tilt = self.create_publisher(Float64, '/tilt_controller/command', 10)

        # Parametri di default per il servo pan/tilt
        self.servomaxx = 1023   # Massima rotazione servo orizzontale (x)
        self.servomaxy = 1023   # Massima rotazione servo verticale (y)
        self.servomin = 0       # Minima rotazione servo
        self.center_pos_x = 512  # Posizione centrale servo orizzontale (x)
        self.center_pos_y = 512  # Posizione centrale servo verticale (y)
        self.servo_step_distancex = 5  # Passi di rotazione servo (x)
        self.servo_step_distancey = 5  # Passi di rotazione servo (y)
        self.current_pos_x = float(self.center_pos_x)
        self.current_pos_y = float(self.center_pos_y)

        # Calcolo dei margini centrali per il tracciamento
        self.screenmaxx = 640  # Risoluzione massima dello schermo (x)
        self.screenmaxy = 480  # Risoluzione massima dello schermo (y)
        self.center_offset = 100
        self.center_offsety = 60
        self.center_left = (self.screenmaxx / 2) - self.center_offset
        self.center_right = (self.screenmaxx / 2) + self.center_offset
        self.center_up = (self.screenmaxy / 2) - self.center_offsety
        self.center_down = (self.screenmaxy / 2) + self.center_offsety

        # Imposta la posizione iniziale centrale
        self.initial_pose_x = Float64()
        self.initial_pose_x.data = float(self.center_pos_x)

        self.initial_pose_y = Float64()
        self.initial_pose_y.data = float(self.center_pos_y)

        self.dynamixel_control.publish(self.initial_pose_x)
        self.dynamixel_control_tilt.publish(self.initial_pose_y)

    def image_callback(self, msg):
        try:
            # Converti il messaggio ROS in un'immagine OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Prepara l'immagine per il modello di deep learning
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))

        self.net.setInput(blob)
        detections = self.net.forward()

        face_found = False  # Variabile per tenere traccia se un volto è stato trovato

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.5:
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                # Disegna un rettangolo intorno al volto rilevato
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)

                # Calcola il centro del volto rilevato
                face_center_x = (startX + endX) // 2
                face_center_y = (startY + endY) // 2

                # Calcola l'offset rispetto al centro dell'immagine
                offset_x = face_center_x - (w // 2)
                offset_y = face_center_y - (h // 2)

                # Chiama la funzione di tracciamento del volto con gli offset calcolati
                self.track_face(offset_x, offset_y)

                face_found = True  # Indica che un volto è stato trovato

        if not face_found:
            # Se non ci sono volti, puoi decidere se lasciare i servos fermi
            pass

        # Mostra l'immagine con il volto rilevato
        cv2.imshow('Face Recognition', frame)
        cv2.waitKey(1)

    def track_face(self, x, y):
        # Controllo asse X (pan)
        print(x,self.center_left, self.center_right , y,self.center_up ,self.center_down)
        if x < self.center_left:
            self.current_pos_x += self.servo_step_distancex
            if self.current_pos_x <= self.servomaxx and self.current_pos_x >= self.servomin:
                current_pose_x = Float64()
                current_pose_x.data = self.current_pos_x
                self.dynamixel_control.publish(current_pose_x)

        elif x > self.center_right:
            self.current_pos_x -= self.servo_step_distancex
            if self.current_pos_x <= self.servomaxx and self.current_pos_x >= self.servomin:
                current_pose_x = Float64()
                current_pose_x.data = self.current_pos_x
                self.dynamixel_control.publish(current_pose_x)

        # Controllo asse Y (tilt)
        if y < self.center_up:
            self.current_pos_y -= self.servo_step_distancey
            if self.current_pos_y <= self.servomaxy and self.current_pos_y >= self.servomin:
                current_pose_y = Float64()
                current_pose_y.data = self.current_pos_y
                self.dynamixel_control_tilt.publish(current_pose_y)

        elif y > self.center_down:
            self.current_pos_y += self.servo_step_distancey
            if self.current_pos_y <= self.servomaxy and self.current_pos_y >= self.servomin:
                current_pose_y = Float64()
                current_pose_y.data = self.current_pos_y
                self.dynamixel_control_tilt.publish(current_pose_y)


def main(args=None):
    rclpy.init(args=args)
    face_recognition_and_tracking_node = FaceRecognitionAndTrackingNode()
    rclpy.spin(face_recognition_and_tracking_node)
    face_recognition_and_tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
