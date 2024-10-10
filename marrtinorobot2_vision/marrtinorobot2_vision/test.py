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

        # Parametri per il servo pan/tilt
        self.servo_max_x = 1023   # Massima rotazione servo orizzontale (x)
        self.servo_max_y = 1023   # Massima rotazione servo verticale (y)
        self.servo_min = 0        # Minima rotazione servo
        self.center_pos_x = 512   # Posizione centrale servo orizzontale (x)
        self.center_pos_y = 512   # Posizione centrale servo verticale (y)

        # Risoluzione dell'immagine della camera
        self.screen_width = 640
        self.screen_height = 480

        # Stato corrente dei servomotori
        self.current_pos_x = float(self.center_pos_x)
        self.current_pos_y = float(self.center_pos_y)

        # Imposta la posizione iniziale dei servo
        self.set_initial_position()

    def set_initial_position(self):
        # Imposta i servo alla posizione centrale
        initial_pose_x = Float64()
        initial_pose_x.data = float(self.center_pos_x)
        initial_pose_y = Float64()
        initial_pose_y.data = float(self.center_pos_y)
        
        self.dynamixel_control.publish(initial_pose_x)
        self.dynamixel_control_tilt.publish(initial_pose_y)

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
            # Se non ci sono volti, i servo rimangono nella loro posizione corrente
            pass

        # Mostra l'immagine con il volto rilevato
        cv2.imshow('Face Recognition', frame)
        cv2.waitKey(1)

    def track_face(self, offset_x, offset_y):
        # Trasforma l'offset in movimenti proporzionali dei servo
        move_x = offset_x * (512 / (self.screen_width // 2))  # Proporzione rispetto alla risoluzione
        move_y = offset_y * (512 / (self.screen_height // 2))

        # Aggiorna la posizione attuale dei servo con il movimento calcolato
        self.current_pos_x = self.center_pos_x - move_x
        self.current_pos_y = self.center_pos_y - move_y

        # Clamping per evitare che i servo escano dai loro limiti
        self.current_pos_x = max(self.servo_min, min(self.current_pos_x, self.servo_max_x))
        self.current_pos_y = max(self.servo_min, min(self.current_pos_y, self.servo_max_y))

        # Pubblica le nuove posizioni dei servo
        current_pose_x = Float64()
        current_pose_x.data = self.current_pos_x
        self.dynamixel_control.publish(current_pose_x)

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
