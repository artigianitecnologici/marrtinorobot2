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
        # pan_controller 
        self.servo_step_distancex = 2  # Passi di rotazione servo (x)
        self.servo_step_distancey = 2  # Passi di rotazione servo (y)
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

        # Aggiungi le linee delle ordinate X e Y con le etichette
        center_x, center_y = w // 2, h // 2
        # Disegna la linea X
        cv2.line(frame, (0, center_y), (w, center_y), (0, 255, 0), 2)
        cv2.putText(frame, 'X', (w - 20, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Disegna la linea Y
        cv2.line(frame, (center_x, 0), (center_x, h), (255, 0, 0), 2)
        cv2.putText(frame, 'Y', (center_x + 10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # Aggiungi una scala per l'asse X e Y
        scale_length = 50  # Lunghezza della scala in pixel
        for i in range(-2, 3):
            # Scala per l'asse X
            cv2.line(frame, (center_x + i * scale_length, center_y - 5), (center_x + i * scale_length, center_y + 5), (0, 255, 0), 2)
            cv2.putText(frame, f'{i*scale_length}', (center_x + i * scale_length - 10, center_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

            # Scala per l'asse Y
            cv2.line(frame, (center_x - 5, center_y + i * scale_length), (center_x + 5, center_y + i * scale_length), (255, 0, 0), 2)
            cv2.putText(frame, f'{i*scale_length}', (center_x + 10, center_y + i * scale_length + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

        # Disegna i margini centrali per il tracciamento
        # Margine sinistro
        cv2.line(frame, (int(self.center_left), 0), (int(self.center_left), h), (255, 255, 0), 2)
        cv2.putText(frame, 'Left Margin', (int(self.center_left) - 60, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # Margine destro
        cv2.line(frame, (int(self.center_right), 0), (int(self.center_right), h), (255, 255, 0), 2)
        cv2.putText(frame, 'Right Margin', (int(self.center_right) - 60, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # Margine superiore
        cv2.line(frame, (0, int(self.center_up)), (w, int(self.center_up)), (255, 255, 0), 2)
        cv2.putText(frame, 'Upper Margin', (30, int(self.center_up) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # Margine inferiore
        cv2.line(frame, (0, int(self.center_down)), (w, int(self.center_down)), (255, 255, 0), 2)
        cv2.putText(frame, 'Lower Margin', (30, int(self.center_down) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # Mostra l'immagine con il volto rilevato, le ordinate e i margini centrali
        cv2.imshow('Face Recognition', frame)
        cv2.waitKey(1)

   
    def track_face(self, x, y):
        # Imposta una "dead zone" per ridurre l'oscillazione
        dead_zone_x = 20  # Definisci la zona morta per l'asse X (orizzontale)
        dead_zone_y = 20  # Definisci la zona morta per l'asse Y (verticale)

        # Stampa per debug (puoi rimuovere in seguito)
        print(f"Offset X: {x}, Offset Y: {y}")
    
        # Controllo asse X (pan) - Muove il servo solo se il volto è fuori dalla zona morta
        if x < -dead_zone_x:  # Se il volto è a sinistra della zona morta
            self.current_pos_x += self.servo_step_distancex
            if self.current_pos_x <= self.servomaxx and self.current_pos_x >= self.servomin:
                current_pose_x = Float64()
                current_pose_x.data = self.current_pos_x
                self.dynamixel_control.publish(current_pose_x)

        elif x > dead_zone_x:  # Se il volto è a destra della zona morta
            self.current_pos_x -= self.servo_step_distancex
            if self.current_pos_x <= self.servomaxx and self.current_pos_x >= self.servomin:
                current_pose_x = Float64()
                current_pose_x.data = self.current_pos_x
                self.dynamixel_control.publish(current_pose_x)

        # Controllo asse Y (tilt) - Muove il servo solo se il volto è fuori dalla zona morta
        if y < -dead_zone_y:  # Se il volto è sopra la zona morta
            self.current_pos_y -= self.servo_step_distancey
            if self.current_pos_y <= self.servomaxy and self.current_pos_y >= self.servomin:
                current_pose_y = Float64()
                current_pose_y.data = self.current_pos_y
                self.dynamixel_control_tilt.publish(current_pose_y)

        elif y > dead_zone_y:  # Se il volto è sotto la zona morta
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
