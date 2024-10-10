import tkinter as tk
import math

class ExcitedEyesAnimation:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(root, width=400, height=200, bg='black')
        self.canvas.pack()

        self.animation_step = 0
        self.is_animating = False
        self.blinking = False

        # Disegna gli occhi emozionati iniziali (pillole verticali)
        self.draw_excited_eyes(0)
        
    def draw_excited_eyes(self, step):
        self.canvas.delete("all")  # Pulizia del canvas

        # Movimento oscillante verticale per simulare eccitazione
        oscillation = math.sin(step * 0.3) * 5  # Oscillazione leggera verticale

        # Disegna occhi emozionati con forma a "pillola verticale"
        if step % 6 == 0 and not self.blinking:
            # Palpebre chiuse (blink)
            self.canvas.create_line(90, 100, 130, 100, fill="#00FFFF", width=5)  # Occhio sinistro chiuso
            self.canvas.create_line(270, 100, 310, 100, fill="#00FFFF", width=5)  # Occhio destro chiuso
            self.blinking = True
        else:
            # Occhi aperti, forma a pillola verticale con oscillazione verticale
            self.canvas.create_oval(90, 70 + oscillation, 130, 130 + oscillation, fill="#00FFFF", outline="")  # Sinistro
            self.canvas.create_oval(270, 70 + oscillation, 310, 130 + oscillation, fill="#00FFFF", outline="")  # Destro
            self.blinking = False

    def animate_excited_eyes(self):
        self.is_animating = True
        self.animation_step = 0
        self.animate_step()

    def animate_step(self):
        if self.animation_step <= 20:
            # Disegna gli occhi per il passo attuale
            self.draw_excited_eyes(self.animation_step)
            self.animation_step += 1
            
            # Chiama la funzione dopo 100ms per animare al prossimo step
            self.root.after(100, self.animate_step)
        else:
            self.is_animating = False  # Fine dell'animazione

# Creazione della finestra principale
root = tk.Tk()
root.title("Robot Excited Eyes Animation - Pillole Verticali")

# Creiamo un'istanza dell'animazione degli occhi emozionati
excited_eyes = ExcitedEyesAnimation(root)

# Bottone per avviare l'animazione
start_button = tk.Button(root, text="Anima Occhi Emozionati", command=excited_eyes.animate_excited_eyes)
start_button.pack()

# Avvio dell'interfaccia Tkinter
root.mainloop()
