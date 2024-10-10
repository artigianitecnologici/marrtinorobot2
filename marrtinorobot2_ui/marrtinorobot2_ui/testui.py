import time
import itertools
import math
from tkinter import *
import random  # Import random for random blinking


class FacePlayer():

    def __init__(self, root):

        self.width = 1024
        self.height = 1024

        self.nomwidth = 800
        self.nomheight = 480

        self.eye_pos_x = 0  # Position for eye movement left/right
        self.eye_pos_y = 0  # Position for eye movement up/down
        self.squint_amount = 0

        self.eye_width = self.nom_x(100)
        self.eye_height = self.nom_y(280)
        self.blink_factor = 0.002
        self.x_centre = self.nom_x(400)
        self.y_centre = self.nom_y(140)
        self.iris_size = self.nom_x(150)
        self.pupil_size = self.nom_x(76)
        self.highlight_size = self.nom_x(20)
        self.highlight_offset = self.nom_x(25)
        self.eye_spacing = self.nom_x(310)

        self.left_centre = self.x_centre - self.eye_spacing / 2
        self.right_centre = self.x_centre + self.eye_spacing / 2
        self.eye_top = self.y_centre - self.eye_height / 2
        self.eye_bottom = self.y_centre + self.eye_height / 2
        self.leye_left = self.left_centre - self.eye_width / 2
        self.leye_right = self.left_centre + self.eye_width / 2
        self.reye_left = self.right_centre - self.eye_width / 2
        self.reye_right = self.right_centre + self.eye_width / 2

        self.tk = Frame(root)
        self.tk.pack(expand=True, fill="both")

        self.canvas = Canvas(self.tk, width=self.width, height=self.height)
        self.canvas.pack(anchor='nw')
        self.canvas.configure(bg='black')  # Background set to black

        self.col_a = "#ff7b00"
        self.col_b = "#a65000"

        # Create eyes (white)
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
                                                   fill="white",
                                                   outline="")

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
                                                   fill="white",
                                                   outline="")

        # Move eyes randomly
        self.run()

    def nom_x(self, value):
        return value / self.nomwidth * self.width

    def nom_y(self, value):
        return value / self.nomheight * self.height

    # Move the irises and pupils
    def move_eyes(self, dx, dy):
        # Update position with bounds
        self.eye_pos_x = max(min(dx, 50), -50)
        self.eye_pos_y = max(min(dy, 20), -20)

        # Move left eye
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

        # Move right eye
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

    # Expression control: You can implement various facial expressions by moving eyelids and eye positions.
    def surprise_expression(self):
        self.move_eyes(0, -15)  # Move eyes slightly up for a surprised look

    def angry_expression(self):
        self.move_eyes(-20, 0)  # Eyes squinted to the side to simulate anger

    def neutral_expression(self):
        self.move_eyes(0, 0)  # Default neutral expression

    def random_eye_movement(self):
        # Randomly move eyes within defined bounds
        dx = random.randint(-50, 50)
        dy = random.randint(-20, 20)
        self.move_eyes(dx, dy)

    def run(self):
        self.tk.update()

        # Simulate random eye movement
        self.random_eye_movement()

        self.tk.after(500, self.run)  # Update every 500 milliseconds


if __name__ == "__main__":
    root = Tk()
    face_player = FacePlayer(root)
    root.mainloop()
