import time
import itertools
import math
from tkinter import *
import random
import rclpy  # Import rclpy for ROS 2
from rclpy.node import Node  # Import Node class
from std_msgs.msg import String, Float32MultiArray  # Import Float32MultiArray for face position

# ROS 2 Node class for controlling the face
class FacePlayerNode(Node):

    def __init__(self):
        super().__init__('face_player_node')

        # FacePlayer logic initialization
        self.canvas_size = 1024  # Make the canvas square (width and height equal)
        self.nominal_size = 1024  # Nominal size, square too
        self.eye_pos_x = 0
        self.eye_pos_y = 0
        self.squint_amount = 0

        # Set eye width and height to be equal for a square appearance
        self.eye_size = self.nom_x(150)  # Increase eye size to fit the square canvas
        self.blink_factor = 0.002
        self.x_centre = self.nom_x(400)
        self.y_centre = self.nom_y(400)  # Centre vertically and horizontally for the square
        self.iris_size = self.nom_x(150)
        self.pupil_size = self.nom_x(76)
        self.highlight_size = self.nom_x(20)
        self.highlight_offset = self.nom_x(25)
        self.eye_spacing = self.nom_x(310)
        self.left_centre = self.x_centre - self.eye_spacing / 2
        self.right_centre = self.x_centre + self.eye_spacing / 2

        # Initialize Tkinter
        self.root = Tk()
        self.tk_frame = Frame(self.root)
        self.tk_frame.pack(expand=True, fill="both")
        self.canvas = Canvas(self.tk_frame, width=self.canvas_size, height=self.canvas_size)  # Make the canvas square
        self.canvas.pack(anchor='nw')
        self.canvas.configure(bg='black')

        # Eye colors and drawing
        self.init_eyes()

        # ROS 2 subscriber to control expressions
        self.expression_subscription = self.create_subscription(
            String, 'face_expression', self.expression_callback, 10)
        # ROS 2 subscriber for face position (x, y)
        self.face_position_subscription = self.create_subscription(
            Float32MultiArray, 'face_position', self.face_position_callback, 10)

        # Timer to handle face updates
        self.timer = self.create_timer(0.5, self.run)

    def nom_x(self, value):
        return value / self.nominal_size * self.canvas_size

    def nom_y(self, value):
        return value / self.nominal_size * self.canvas_size

    def init_eyes(self):
        iris_colour = "white"
        self.iris_l = self.canvas.create_oval(self.left_centre - self.iris_size / 2,
                                              self.y_centre - self.iris_size / 2,
                                              self.left_centre + self.iris_size / 2,
                                              self.y_centre + self.iris_size / 2,
                                              fill=iris_colour)
        self.pup_l = self.canvas.create_oval(self.left_centre - self.pupil_size / 2,
                                             self.y_centre - self.pupil_size / 2,
                                             self.left_centre + self.pupil_size / 2,
                                             self.y_centre + self.pupil_size / 2,
                                             fill="black")
        self.highlight_l = self.canvas.create_oval(self.left_centre - self.highlight_size / 2 + self.highlight_offset,
                                                   self.y_centre - self.highlight_size / 2 - self.highlight_offset,
                                                   self.left_centre + self.highlight_size / 2 + self.highlight_offset,
                                                   self.y_centre + self.highlight_size / 2 - self.highlight_offset,
                                                   fill="white", outline="")
        self.iris_r = self.canvas.create_oval(self.right_centre - self.iris_size / 2,
                                              self.y_centre - self.iris_size / 2,
                                              self.right_centre + self.iris_size / 2,
                                              self.y_centre + self.iris_size / 2,
                                              fill=iris_colour)
        self.pup_r = self.canvas.create_oval(self.right_centre - self.pupil_size / 2,
                                             self.y_centre - self.pupil_size / 2,
                                             self.right_centre + self.pupil_size / 2,
                                             self.y_centre + self.pupil_size / 2,
                                             fill="black")
        self.highlight_r = self.canvas.create_oval(self.right_centre - self.highlight_size / 2 + self.highlight_offset,
                                                   self.y_centre - self.highlight_size / 2 - self.highlight_offset,
                                                   self.right_centre + self.highlight_size / 2 + self.highlight_offset,
                                                   self.y_centre + self.highlight_size / 2 - self.highlight_offset,
                                                   fill="white", outline="")

    def move_eyes(self, dx, dy):
        self.eye_pos_x = max(min(dx, 50), -50)
        self.eye_pos_y = max(min(dy, 50), -50)  # Increase the max movement upwards to 50
        # Update left eye position
        self.canvas.coords(self.iris_l,
                           self.left_centre - self.iris_size / 2 + self.eye_pos_x,
                           self.y_centre - self.iris_size / 2 + self.eye_pos_y,
                           self.left_centre + self.iris_size / 2 + self.eye_pos_x,
                           self.y_centre + self.iris_size / 2 + self.eye_pos_y)
        self.canvas.coords(self.pup_l,
                           self.left_centre - self.pupil_size / 2 + self.eye_pos_x,
                           self.y_centre - self.pupil_size / 2 + self.eye_pos_y,
                           self.left_centre + self.pupil_size / 2 + self.eye_pos_x,
                           self.y_centre + self.pupil_size / 2 + self.eye_pos_y)
        self.canvas.coords(self.highlight_l,
                           self.left_centre - self.highlight_size / 2 + self.highlight_offset + self.eye_pos_x,
                           self.y_centre - self.highlight_size / 2 - self.highlight_offset + self.eye_pos_y,
                           self.left_centre + self.highlight_size / 2 + self.highlight_offset + self.eye_pos_x,
                           self.y_centre + self.highlight_size / 2 - self.highlight_offset + self.eye_pos_y)
        # Update right eye position
        self.canvas.coords(self.iris_r,
                           self.right_centre - self.iris_size / 2 + self.eye_pos_x,
                           self.y_centre - self.iris_size / 2 + self.eye_pos_y,
                           self.right_centre + self.iris_size / 2 + self.eye_pos_x,
                           self.y_centre + self.iris_size / 2 + self.eye_pos_y)
        self.canvas.coords(self.pup_r,
                           self.right_centre - self.pupil_size / 2 + self.eye_pos_x,
                           self.y_centre - self.pupil_size / 2 + self.eye_pos_y,
                           self.right_centre + self.pupil_size / 2 + self.eye_pos_x,
                           self.y_centre + self.pupil_size / 2 + self.eye_pos_y)
        self.canvas.coords(self.highlight_r,
                           self.right_centre - self.highlight_size / 2 + self.highlight_offset + self.eye_pos_x,
                           self.y_centre - self.highlight_size / 2 - self.highlight_offset + self.eye_pos_y,
                           self.right_centre + self.highlight_size / 2 + self.highlight_offset + self.eye_pos_x,
                           self.y_centre + self.highlight_size / 2 - self.highlight_offset + self.eye_pos_y)

    def face_position_callback(self, msg):
        face_x = msg.data[0]  # Extract the x-coordinate
        face_y = msg.data[1]  # Extract the y-coordinate
        
        # Map face coordinates (-1 to 1) to eye movement range
        eye_dx = face_x * 50  # Max horizontal eye movement
        eye_dy = face_y * 50  # Increase max vertical movement to 50
        
        self.move_eyes(eye_dx, eye_dy)

    def expression_callback(self, msg):
        expression = msg.data
        if expression == 'surprise':
            self.surprise_expression()
        elif expression == 'angry':
            self.angry_expression()
        else:
            self.neutral_expression()

    def run(self):
        self.root.update_idletasks()
        self.root.update()


def main(args=None):
    rclpy.init(args=args)
    face_node = FacePlayerNode()
    rclpy.spin(face_node)
    face_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()