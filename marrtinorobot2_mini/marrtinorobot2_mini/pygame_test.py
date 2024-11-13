import time
import os
import pygame
import multiprocessing
import logging
from PIL import Image

# Frame count dictionary for various emotions
frame_count = {
    'blink': 39, 'happy': 60, 'sad': 47, 'dizzy': 67, 'excited': 24,
    'neutral': 61, 'happy2': 20, 'angry': 20, 'happy3': 26, 'bootup3': 124, 'blink2': 20
}

# Define available emotions
emotion = ['angry', 'sad', 'excited']
normal = ['neutral', 'blink2']

# Initialize multiprocessing queue and event
q = multiprocessing.Queue()
event = multiprocessing.Event()

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((800, 480))  # Adjust resolution as needed
pygame.display.set_caption('Emotion Display')
pygame.mixer.init()

logging.basicConfig(level=logging.INFO)

def sound(emotion):
    """Play sound associated with the emotion."""
    try:
        sound_path = f"./sound/{emotion}.wav"
        if os.path.exists(sound_path):
            sound = pygame.mixer.Sound(sound_path)
            sound.play()
        else:
            logging.warning(f"Sound file for {emotion} not found.")
    except Exception as e:
        logging.error(f"Error playing sound for {emotion}: {e}")

def show(emotion, count):
    """Display frames for the specified emotion on the main screen using pygame."""
    for _ in range(count):
        try:
            for i in range(frame_count.get(emotion, 0)):
                image_path = f'./emotions/{emotion}/frame{i}.png'
                if os.path.exists(image_path):
                    image = pygame.image.load(image_path)
                    image = pygame.transform.scale(image, (800, 480))  # Adjust size as needed
                    screen.blit(image, (0, 0))
                    pygame.display.update()
                    time.sleep(0.05)  # Adjust frame delay as necessary
                else:
                    logging.warning(f"Frame {i} for emotion {emotion} not found.")
        except Exception as e:
            logging.error(f"Error displaying {emotion}: {e}")
            pygame.quit()
            exit()

def bootup():
    """Display bootup sequence and a simple blink effect."""
    show('bootup3', 1)
    p2 = multiprocessing.Process(target=show, args=('blink2', 3))
    p3 = multiprocessing.Process(target=sound, args=('start',3))
    p2.start()
    p3.start()
    p2.join()
    p3.join()

if __name__ == '__main__':
    # Initial bootup display
    bootup()
    
    # Main loop to handle emotions
    try:
        while True:
            if event.is_set():
                event.clear()
                try:
                    emotion = q.get_nowait()
                    print(f"Displaying emotion: {emotion}")
                    p2 = multiprocessing.Process(target=show, args=(emotion, 4))
                    p3 = multiprocessing.Process(target=sound, args=(emotion,4))
                    p2.start()
                    p3.start()
                    p2.join()
                    p3.join()
                except multiprocessing.queues.Empty:
                    logging.info("Queue is empty, no emotion to display.")
            else:
                neutral = normal[0]
                p2 = multiprocessing.Process(target=show, args=(neutral, 4), name='neutral_display')
                p2.start()
                p2.join()
    except KeyboardInterrupt:
        logging.info("Exiting gracefully")
        pygame.quit()
